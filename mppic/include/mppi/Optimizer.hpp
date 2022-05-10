#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.h"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "std_msgs/msg/float64.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>

#include "xtensor/xarray.hpp"
#include <xtensor/xview.hpp>

// slopeRoughnessAtPose(const T & x, const T & y, T & slp, T & rgh) const;

//   void evalSlopeTraversabilityCost

namespace mppi::optimization
{

	const std::string COSTMAP_METHOD = "costmap";
	const std::string SLOPE_ROUGHNESS_METHOD = "slope_roughness";
	const std::string SLOPE_TRAVERSABILITY_METHOD = "slope_traversability";

	template <typename T,
			  typename Model = xt::xtensor<T, 2>(const xt::xtensor<T, 2> &)>
	class Optimizer
	{
	public:
		Optimizer() = default;
		~Optimizer() = default;

		void on_configure(
			const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
			const std::string &node_name,
			const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
			const std::shared_ptr<grid_map::GridMap> &grid_map,
			Model &&model);

		void on_cleanup() {}
		void on_activate() {}
		void on_deactivate() {}

		/**
		 * @brief Calculates control for robot in position robot_pose with velocity robot_speed based on plan
		 * 
		 * @param robot_pose current robot position
		 * @param robot_speed current robot velocity
		 * @param plan current global path
		 * @return new robot velocity
		 */
		auto evalNextBestControl(
			const geometry_msgs::msg::PoseStamped &robot_pose,
			const geometry_msgs::msg::Twist &robot_speed,
			const nav_msgs::msg::Path &plan)
			-> geometry_msgs::msg::TwistStamped;

		/**
		 * @brief Returns the Generated Trajectories object
		 * 
		 * @return tensor of shape [ batch_size, time_steps, 3 ]  where 3 stands for x, y, yaw
		 */
		auto getGeneratedTrajectories() const
			-> xt::xtensor<T, 3>
		{
			return generated_trajectories_;
		}

		/**
		 * @brief Applies the velocity sequence to the batch
		 * 
		 * @param[in] velocities_sequence tensor of shape [ time_steps, 2 ] where 2 stands for robot linear and angluar velocities
		 * @param[in] initial_speed velocity in form of ROS Twist
		 * @param[out] batch tensor of shape [ time_steps, 5 ] where 5 stands for robot linear, angluar velocities, linear, angular control velocities, dt (time on which this control will be applied)
		 */
		void propagateSequenceVelocities(
			const auto &velocities_sequence,
			const geometry_msgs::msg::Twist &initial_speed,
			auto &batch) const;

		/**
		 * @brief Creates trajectory based on control sequence and initial state
		 * 
		 * @param robot_pose current robot position
		 * @param robot_speed current robot velocity
		 * 
		 * @return tensor of shape [ time_steps, 3 ]  where 3 stands for x, y, yaw
		 */
		auto evalTrajectoryFromControlSequence(
			const geometry_msgs::msg::PoseStamped &robot_pose,
			const geometry_msgs::msg::Twist &robot_speed) const
			-> xt::xtensor<T, 2>;

	private:
		void getParams();
		void resetBatches();

		/**
		 * @brief Invokes generateNoisedControlBatches, assign result tensor to batches_ controls dimensions
		 * and integrate recieved controls in trajectories
		 * @param robot_pose current robot position
		 * @param robot_speed current robot velocity
		 * 
		 * @return trajectories: tensor of shape [ batch_size_, time_steps_, 3 ]  where 3 stands for x, y, yaw
		 */
		auto generateNoisedTrajectories(
			const geometry_msgs::msg::PoseStamped &robot_pose,
			const geometry_msgs::msg::Twist &robot_speed)
			-> xt::xtensor<T, 3>;

		/**
		 * @brief Generates random controls by gaussian noise with mean in
		 * control_sequence_
		 *
		 * @return Control batches tensor of shape [ batch_size_, time_steps_, 2] where 2 stands for v, w
		 */
		auto generateNoisedControlBatches() const
			-> xt::xtensor<T, 3>;

		/**
		 * @brief Applies dynamic constraints of the robot to the sequence of controls  
		 */
		void applyControlConstraints();

		/**
		 * @brief Adds a set of predefined controls (for straight trajectories) to the random generated set of controls
		 * 
		 * @param[in, out] v_noises linear control sequences
		 * @param[in, out] w_noises angular control sequences
		 */
		void addStrajghtTrajectories(xt::xtensor<T, 3> &v_noises, xt::xtensor<T, 3> &w_noises) const;

		/**
		 * @brief Invokes setBatchesInitialVelocities and propagateBatchesVelocitiesFromInitials
		 *
		 * @param robot_speed current robot speed
		 */
		void evalBatchesVelocities(const geometry_msgs::msg::Twist &robot_speed);

		/**
		 * @brief Sets the initial velocity for all batches 
		 * 
		 * @param robot_speed initial velocity 
		 */
		void setBatchesInitialVelocities(const geometry_msgs::msg::Twist &robot_speed);

		/**
		 * @brief Predicts and propagates velocities in batches_ using model
		 * for time horizont equal to time_steps_
		 */
		void propagateBatchesVelocitiesFromInitials();

		auto integrateBatchesVelocities(const geometry_msgs::msg::PoseStamped &robot_pose) const
			-> xt::xtensor<T, 3>;

		auto integrateSequence(
			const auto &velocities_sequence,
			const geometry_msgs::msg::PoseStamped &robot_pose) const
			-> xt::xtensor<T, 2>;

		/**
		 * @brief Evaluates cost for each batch
		 *
		 * @param batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ]
		 * where 3 stands for x, y, yaw
		 * @return Cost for each batch, tensor of shape [ batch_size ]
		 */
		auto evalBatchesCosts(
			const xt::xtensor<T, 3> &batches_of_trajectories,
			const nav_msgs::msg::Path &global_plan,
			const geometry_msgs::msg::PoseStamped &robot_pose) const
			-> xt::xtensor<T, 1>;

		/**
		 * @brief Evaluates and adds to cost values related to backward motion (forward motion is safer and more efficient) 
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalBackwardMotionCost(const auto &batches_of_trajectories_points, auto &costs) const;

		/**
		 * @brief Evaluates and adds to cost values related to trajectories path alignment
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[in] global_plan reference trajectory
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalReferenceCost(
			const auto &batches_of_trajectories,
			const auto &global_plan,
			auto &costs) const;

		/**
		 * @brief Evaluates and adds to cost values related to trajectories path alignment using approximate path to segment function
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[in] global_plan reference trajectory
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalApproxReferenceCost(
			const auto &batches_of_trajectories,
			const auto &global_plan,
			auto &costs) const;

		/**
		 * @brief Evaluates and adds to cost values related to goal following
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[in] global_plan reference trajectory
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalGoalCost(
			const auto &batch_of_trajectories,
			const auto &global_plan,
			auto &costs) const;

		/**
		 * @brief Evaluate cost related to obstacle avoidance
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalObstacleCost(const auto &batch_of_trajectories, auto &costs) const;

		/**
		 * @brief Evaluate cost using Slope-Roughness method
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalSlopeRoughnessCost(const auto &batches_of_trajectories_points, auto &costs) const;

		/**
		 * @brief Calculates robot circular footprint at position (x, y) using footprint_radius parameter
		 * 
		 * @param x x-coordinate of robot position
		 * @param y y-coordinate of robot position
		 * @return tensor of shape [ n, 2], where n -- number of points inside footprint, 2 stands for x, y
		 */
		auto footprintPointsAtPose(const T &x, const T &y) const -> xt::xtensor<T, 2>;

		/**
		 * @brief Calculates Slope-Roughness metric at position (x, y) using footprint_radius parameter
		 * 
		 * @param[in] x x-coordinate of robot position
		 * @param[in] y y-coordinate of robot position
		 * @param[out] slp slope part of metric
		 * @param[out] rgh roughness part of metric
		 * @return it is possible to compute metric at position (x, y).   
		 */
		bool slopeRoughnessAtPose(const T &x, const T &y, T &slp, T &rgh) const;

		/**
		 * @brief Evaluate cost using Slope-Traversability method
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalSlopeTraversabilityCost(const auto &batches_of_trajectories_points, auto &costs) const;

		/**
		 * @brief Computes minimal distance to untraversable area from position (x, y)
		 * 
		 * @param[in] x x-coordinate of robot position
		 * @param[in] y y-coordinate of robot position
		 * @param[in, out] slope_traversability_dist_grid_ matrix of known distance values
		 * @param[out] dist minimal distance to untraversable area from position (x, y)
		 * @return it is possible to compute metric at position (x, y).    
		 */
		bool minDistToUntraversableFromPose(const T &x, const T &y, auto &slope_traversability_dist_grid_, T &dist) const;
		
		/**
		 * @brief Starts Least Squares calculations
		 */
		void fitPlane(
			const xt::xtensor<T, 2> &points,
			T &a,
			T &b,
			T &c,
			T &sum_resid) const;

		/**
		 * @brief Evaluate cost related to robot orientation at goal pose (considered only if robot near last goal in current plan)
		 *
		 * @param[in] batches_of_trajectories batch of trajectories: tensor of shape [ batch_size_, time_steps_, 3 ] where 3 stands for x, y, yaw
		 * @param[in] global_plan reference trajectory
		 * @param[in] robot_pose current robot position
		 * @param[out] costs Cost for each batch, tensor of shape [ batch_size ]
		 */
		void evalGoalAngleCost(
			const auto &batch_of_trajectories,
			const auto &global_plan,
			const geometry_msgs::msg::PoseStamped &robot_pose,
			auto &costs) const;
		
		/**
		 * @brief Returns cost of position (x, y) using costmap2d
		 * 
		 * @param x x-coordinate of robot position
		 * @param y y-coordinate of robot position
		 * @return cost of position (x, y) using costmap2d
		 */
		auto costAtPose(const double &x, const double &y) const -> double;
		
		/**
		 * @brief Checks, that robot is in collision with static obstacle using costmap2d
		 */
		bool inCollision(unsigned char cost) const;

		/**
		 * @brief Update control_sequence_ with weighted by costs batch controls using softmax function
		 *
		 * @param costs batches costs, tensor of shape [ batch_size ]
		 */
		void updateControlSequence(const xt::xtensor<T, 1> &costs);

		/**
		 * @brief Shift control_sequence_ by 1 time step and set last time step to zero
		 */
		void shiftControlSequence();

		/**
		 * @brief Get first control from control_sequence_
		 */
		auto getControlFromSequence(const auto &stamp, const std::string &frame)
			-> geometry_msgs::msg::TwistStamped;

		auto getBatchesControls() const;
		auto getBatchesControls();

		auto getBatchesControlLinearVelocities() const;
		auto getBatchesControlLinearVelocities();

		auto getBatchesControlAngularVelocities() const;
		auto getBatchesControlAngularVelocities();

		auto getBatchesLinearVelocities() const;
		auto getBatchesLinearVelocities();

		auto getBatchesAngularVelocities() const;
		auto getBatchesAngularVelocities();

	private:
		std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
		std::string node_name_;
		std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
		nav2_costmap_2d::Costmap2D *costmap_;
		std::shared_ptr<grid_map::GridMap> grid_map_;

		std::function<Model> model_;

		double inflation_cost_scaling_factor_;
		double inscribed_radius_;
		double inflation_radius_;

		double threshold_to_consider_goal_angle_;
		bool approx_reference_cost_;

		static constexpr int last_dim_size_ = 5;
		static constexpr int control_dim_size_ = 2;

		int batch_size_;
		int time_steps_;
		int iteration_count_;

		double model_dt_;
		double v_std_;
		double w_std_;
		double v_limit_;
		double w_limit_;
		double max_v_acc_;
		double max_w_acc_;
		double temperature_;

		double backward_motion_cost_weight_;
		double reference_cost_power_;
		double reference_cost_weight_;
		double obstacle_cost_power_;
		double obstacle_cost_weight_;
		double goal_cost_power_;
		double goal_cost_weight_;
		double goal_angle_cost_power_;
		double goal_angle_cost_weight_;

		double slope_cost_power_;
		double slope_cost_weight_;
		double slope_crit_;

		double roughness_cost_power_;
		double roughness_cost_weight_;
		double roughness_crit_;

		double slope_traversability_delta_;
		double slope_traversability_alpha_;
		double slope_traversability_lambda_;

		double slope_traversability_cost_power_;
		double slope_traversability_cost_weight_;

		std::string obstacle_avoidance_method_;
		double footprint_radius_;
		bool predefined_trajectories_on_cold_start_;

		rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr eval_time_publisher_;

		/**
		 * @batches_ tensor of shape [ batch_size, time_steps, 5 ] where 5 stands for
		 * robot linear, angluar velocities, linear control, angular control velocities, dt (time on which this control will be applied)
		 */
		xt::xtensor<T, 3> batches_;
		xt::xtensor<T, 3> generated_trajectories_;
		xt::xtensor<T, 2> control_sequence_;

		rclcpp::Logger logger_{rclcpp::get_logger("MPPI Optimizer")};
	};

} // namespace mppi::optimization