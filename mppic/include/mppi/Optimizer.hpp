#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "xtensor/xadapt.hpp"
#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xslice.hpp"
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "mppi/Utils.hpp"

namespace ultra::mppi::optimization {

template <
  typename T, 
  typename Tensor = xt::xarray<T> 
>
class Optimizer {
public:
  using ManagedNode = rclcpp_lifecycle::LifecycleNode;
  using Costmap2DROS = nav2_costmap_2d::Costmap2DROS;
  using TfBuffer = tf2_ros::Buffer;

  using TwistStamped = geometry_msgs::msg::TwistStamped;
  using Twist = geometry_msgs::msg::Twist;
  using PoseStamped = geometry_msgs::msg::PoseStamped;
  using Path = nav_msgs::msg::Path;

  Optimizer() = default;
  ~Optimizer() = default;

  template<typename Callable>
  Optimizer(
      ManagedNode::SharedPtr const& parent, 
      std::string const& node_name, 
      std::shared_ptr<TfBuffer> const &tf,
      std::shared_ptr<Costmap2DROS> const& costmap_ros,
      Callable&& model) {

      m_parent = parent;
      m_costmap_ros_ = costmap_ros;
      m_tf_ = tf;
      m_node_name_ = node_name;

      m_model = model;

      using namespace utils;
      getParam("model_dt", 0.1, m_parent, m_model_dt);
      getParam("time_steps", 20, m_parent, m_time_steps);
      getParam("batch_size", 100, m_parent, m_batch_size);
      getParam("std_v", 0.1, m_parent, m_std_v);
      getParam("std_w", 0.1, m_parent, m_std_w);
      getParam("limit_v", 0.5, m_parent, m_limit_v);
      getParam("limit_w", 1.0, m_parent, m_limit_w);
      getParam("iteration_count", 2, m_parent, m_iterations_count);
      getParam("lookahead_dist", 1.2, m_parent, m_lookahead_dist);
      getParam("temperature", 0.25, m_parent, m_temperature);
      resetBatches();
  }

  /**
   * @brief Evaluate next control
   *
   * @param Pose current pose of the robot
   * @param Twist Current speed of the rosbot
   * @param Path Current global path
   * @return Next control
   */
  auto evalNextControl(PoseStamped const& pose, Twist const& twist)
  -> TwistStamped {
    (void)pose;
    (void)twist;

    for (int i = 0; i < m_iterations_count; ++i) {
      Tensor trajectories = generateNoisedTrajectoryBatches(pose, twist);
      /* Tensor costs = evalBatchesCosts(trajectories);  // TODO */
      /* updateControlSequence(costs); // NOT TESTED */
    }

    return TwistStamped{}; // TODO
  };

  void setPlan(Path const &path) { m_global_plan_ = path; }

  void resetBatches() {
    m_batches = xt::zeros<float>({m_batch_size, m_time_steps, m_last_dim});
    xt::view(m_batches, xt::all(), xt::all(), 4) = m_model_dt;

    m_control_sequence = xt::zeros<float>({m_time_steps, m_control_dim});
  }

private:
  void updateControlSequence(Tensor costs) {
    costs = costs - xt::amin(costs);
    auto exponents = xt::exp(-1 / m_temperature * costs);
    auto softmaxes = exponents / xt::sum(exponents); // Shape = [ batch_size ]
    auto values = Tensor(xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis()));

    m_control_sequence = getControlBatches() * xt::sum(softmaxes, 0);
  }

  Tensor evalBatchesCosts(Tensor const& trajectory_batches) const {
    return m_cost(trajectory_batches, m_global_plan_);
  }


  Tensor generateNoisedTrajectoryBatches(PoseStamped const& pose, Twist const& twist) {
    getControlBatches() = generateNoisedControlBatches();
    applyControlConstraints();
    setBatchesVelocity(twist);
    return integrateVelocityBatches(pose);
  }

  void applyControlConstraints(){
    xt::clip(getLinearVelocitControlBatches(), -m_limit_v, m_limit_v); // TODO Does it clip tensor itself or do i need to assign?
    xt::clip(getAngularVelocityControlBatches(), -m_limit_w, m_limit_w);
  }

  void setBatchesVelocity(Twist const& twist) {
    setBatchesInitialVelocities(twist);
    propagateBatchesVelocityFromInitials();
  }

  void setBatchesInitialVelocities(Twist const& twist) {
    xt::view(m_batches, xt::all(), 0, 0) = twist.linear.x;
    xt::view(m_batches, xt::all(), 0, 1) = twist.angular.z;
  }

  void propagateBatchesVelocityFromInitials() {
    using namespace xt::placeholders;

    for (int t = 0; t < m_time_steps - 1; t++) {
      auto curr_batch = xt::view(m_batches, xt::all(), t); // -> batch x 5
      auto predicted_velocities = m_model(curr_batch); // -> batch x 2
      auto next_batch_velocities = xt::view(m_batches, xt::all(), t + 1, xt::range(_, 2));  // batch x 2

      next_batch_velocities = predicted_velocities;
    }
  }

  decltype(auto) getControlBatches() {
    return xt::view(m_batches, xt::all(), xt::all(), xt::range(2, 4));
  }

  decltype(auto) getLinearVelocitControlBatches() {
    return xt::view(m_batches, xt::all(), xt::all(), 2);
  }

  decltype(auto) getAngularVelocityControlBatches() {
    return xt::view(m_batches, xt::all(), xt::all(), 3);
  }

  auto generateNoisedControlBatches()
  -> Tensor {

    auto v_noises = xt::random::randn<T>({m_batch_size, m_time_steps, 1}, 0.0, m_std_v);
    auto w_noises = xt::random::randn<T>({m_batch_size, m_time_steps, 1}, 0.0, m_std_w);

    return m_control_sequence + xt::concatenate(xtuple(v_noises, w_noises), 2);
  }

  auto integrateVelocityBatches(PoseStamped const& robot_pose) const
  -> Tensor {

    using namespace xt::placeholders;

    double robot_x = robot_pose.pose.position.x;
    double robot_y = robot_pose.pose.position.y;
    double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);

    auto v = xt::view(m_batches, xt::all(), xt::all(), 0);
    auto w = xt::view(m_batches, xt::all(), xt::all(), 1);

    auto yaw = xt::cumsum(w * m_model_dt, 1);
    yaw -= xt::view(yaw, xt::all(), xt::range(_, 1));

    yaw += robot_yaw;

    auto x = xt::cumsum(v * xt::cos(yaw) * m_model_dt, 1);
    auto y = xt::cumsum(v * xt::sin(yaw) * m_model_dt, 1);

    x += robot_x - xt::view(x, xt::all(), xt::range(_, 1) );
    y += robot_y - xt::view(x, xt::all(), xt::range(_, 1) );

    return xt::concatenate(xtuple(
      xt::view(x, xt::all(), xt::all(), xt::newaxis()),
      xt::view(y, xt::all(), xt::all(), xt::newaxis()),
      xt::view(yaw, xt::all(), xt::all(), xt::newaxis())
      ), 2);
  }

public:
  ManagedNode::SharedPtr m_parent;
  Path m_global_plan_;
  std::string m_node_name_;
  std::shared_ptr<Costmap2DROS> m_costmap_ros_;
  std::shared_ptr<TfBuffer> m_tf_;

  static int constexpr m_last_dim = 5;
  static int constexpr m_control_dim = 2;
  int m_time_steps;
  int m_batch_size;

  double m_model_dt;
  double m_std_v;
  double m_std_w;
  double m_limit_w;
  double m_limit_v;

  int m_iterations_count;
  double m_lookahead_dist;

  double m_temperature;

  Tensor m_batches;
  Tensor m_control_sequence;

  using model_t = Tensor(Tensor);
  std::function<model_t> m_model;

  using cost_t = Tensor(Tensor const&, Path const&, Costmap2DROS const&);
  std::function<cost_t> m_cost;
};

} // namespace ultra::mppi
