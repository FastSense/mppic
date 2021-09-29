#include "grid_map_core/TypeDefs.hpp"
#include <string>
#include <cmath>
#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif
#include <catch2/catch.hpp>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "mppi/Models.hpp"
#include "mppi/impl/Optimizer.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include <iostream>


/**
 * Adds some parameters for the optimizer to a special container.
 *
 * @param params_ container for optimizer's parameters.
*/
void setUpOptimizerParams(std::vector<rclcpp::Parameter> &params_){
  params_.push_back(rclcpp::Parameter("TestNode.iteration_count", 5));
  params_.push_back(rclcpp::Parameter("TestNode.lookahead_dist", 5));
  params_.push_back(rclcpp::Parameter("TestNode.time_steps", 30));
}


/*!
* Prints map in to cout
* @param grid_map map to be printed.
* @param layer_name name of the map layer.
*/
void printGridMapLayer(grid_map::GridMap & grid_map, const std::string & layer_name){
  std::cout<<grid_map.get(layer_name)<<std::endl;
}

/*!
* Prints map, trajectory and goal in to cout.
* @param grid_map map to be printed.
* @param layer_name name of the map layer.
* @param trajectory trajectory to be printed.
* @param goal_point target point in world coordinates.
*/
void printGridMapLayerWithTrajectoryAndGoal(grid_map::GridMap & grid_map, 
                                          const std::string & layer_name,
                                          const auto & trajectory,
                                          const geometry_msgs::msg::PoseStamped & goal_point){
  auto matrix = grid_map.get(layer_name);
  double grid_map_resolution = grid_map.getResolution();
  float origin_x = grid_map.getPosition()(0);
  float origin_y = grid_map.getPosition()(1);
  std::cout<<"GridMap \n start point=-100 trajectory=-1 goal point=100 \n"<<std::endl;
  int start_point_x = (trajectory(0, 0) - origin_x)/ grid_map_resolution;
  int start_point_y = (trajectory(0, 1) - origin_y)/ grid_map_resolution;
  matrix(start_point_x, start_point_y) = -100.0;
  for (size_t i = 1; i < trajectory.shape()[0]; ++i){
    int traj_point_x = (trajectory(i, 0) - origin_x)/ grid_map_resolution;
    int traj_point_y = (trajectory(i, 1) - origin_y)/ grid_map_resolution;
    if (traj_point_x!=start_point_x || traj_point_y!=start_point_y){
      matrix(traj_point_x, traj_point_y) = -1.0;
    }
  }

  float goal_ind_x = (goal_point.pose.position.x - origin_x) / grid_map_resolution;
  float goal_ind_y = (goal_point.pose.position.y - origin_y) / grid_map_resolution;
  matrix(goal_ind_x, goal_ind_y) = 100.0;
  std::cout<<matrix<<std::endl;
}


/*!
* Adds a hill to the grid map
* @param grid_map grid map to be updated.
* @param layer_name name of the map layer.
* @param hill_height hill height in meters.
* @param hill_step step of changing the height between cells in meters.
* @param hill_size_m size (length) of the hill in meters.
* @param upper_left_corner the x and y coordinate of the upper left corner of the hill.
*/
void addHillToGridMapLayer(grid_map::GridMap & grid_map, std::string layer_name,
                          float hill_height, float hill_step, float hill_size_m,
                          grid_map::Index upper_left_corner){
  
  double grid_map_resolution = grid_map.getResolution();                    
  float hill_size_cells = hill_size_m / grid_map_resolution;
  auto & layer_matrix = grid_map.get(layer_name);
  float h = hill_step;
  size_t y_limit = upper_left_corner(1)+hill_size_cells/2;
  for (size_t j = upper_left_corner(1); j <= y_limit; j++){
    layer_matrix.block(upper_left_corner(0), j, hill_size_cells, hill_size_cells).setConstant(h);
    upper_left_corner(0) = upper_left_corner(0) + 1;
    hill_size_cells = hill_size_cells - 2;
    if (h + hill_step <= hill_height){
      h = h + hill_step;
    }
  }
}

/*!
* Adds a ramp to the grid map
* @param grid_map grid map to be updated.
* @param layer_name name of the map layer.
* @param ramp_height ramp height in meters.
* @param ramp_step step of changing the height between cells in meters.
* @param ramp_size size (length) of the ramp in meters.
* @param upper_left_corner the x and y coordinate of the upper left corner of the ramp.
*/
void addRampToGridMapLayer(grid_map::GridMap & grid_map, std::string layer_name,
                         float ramp_height, float ramp_step, float ramp_size,
                         grid_map::Index upper_left_corner){

  double grid_map_resolution = grid_map.getResolution();
  float ramp_size_cells = ramp_size / grid_map_resolution;
  auto & layer_matrix = grid_map.get(layer_name);
  float curr_h = ramp_step;

  for (size_t j = upper_left_corner(1); j < upper_left_corner(1) + ramp_size_cells; j++){
    for (size_t i = upper_left_corner(0); i < upper_left_corner(0) + ramp_size_cells; i++){
      layer_matrix.row(j)(i) = curr_h;
    }
    if (curr_h + ramp_step <= ramp_height){
      curr_h = curr_h + ramp_step;
    }
  }
}

/*!
* Сhecks the trajectory for intersections with obstacles
* @param grid_map map to be printed.
* @param layer_name name of the map layer.
* @param robot_clearance the maximum height between the cells of the grid map that the robot can overcome
* @param trajectory trajectory to be printed.
*/
bool InCollison(const grid_map::GridMap & grid_map, 
                              std::string & layer_name,
                              float robot_clearance,
                              const auto & trajectory){

  auto & matrix = grid_map.get(layer_name);
  double grid_map_resolution = grid_map.getResolution();
  float origin_x = grid_map.getPosition()(0);
  float origin_y = grid_map.getPosition()(1);

  int traj_point_x_prev = (trajectory(0, 0) - origin_x)/ grid_map_resolution;
  int traj_point_y_prev = (trajectory(0, 1) - origin_y)/ grid_map_resolution;
  for (size_t i = 1; i < trajectory.shape()[0]; ++i){
    int traj_point_x = (trajectory(i, 0) - origin_x)/ grid_map_resolution;
    int traj_point_y = (trajectory(i, 1) - origin_y)/ grid_map_resolution;
    bool cant_drive = abs(matrix(traj_point_x, traj_point_y) - matrix(traj_point_x_prev, traj_point_y_prev)) > robot_clearance;
    if (cant_drive){
      return true;
    }
    traj_point_x_prev = traj_point_x;
    traj_point_y_prev = traj_point_y;
  }
  return false;
}

TEST_CASE("Optimizer evaluates Next Control", "") {
  using T = float;

  std::string node_name = "TestNode";
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("cost_map_node");
  auto grid_map = std::make_shared<grid_map::GridMap>();
  auto st = rclcpp_lifecycle::State{};

  auto & model = mppi::models::NaiveModel<T>;

  auto optimizer =
    mppi::optimization::Optimizer<T>();

  costmap_ros->on_configure(st);
  optimizer.on_configure(node, node_name, costmap_ros, grid_map, model);
  optimizer.on_activate();

  size_t poses_count = GENERATE(10, 30, 100);

  SECTION("Running evalNextControl") {
    geometry_msgs::msg::Twist twist;

    std::string frame = "odom";
    auto time = node->get_clock()->now();

    auto setHeader = [&](auto && msg) {
        msg.header.frame_id = frame;
        msg.header.stamp = time;
      };

    nav_msgs::msg::Path path;
    geometry_msgs::msg::PoseStamped ps;
    setHeader(ps);
    setHeader(path);

    auto fillPath = [&](size_t count) {
        for (size_t i = 0; i < count; i++) {
          path.poses.push_back(ps);
        }
      };

    fillPath(poses_count);

    CHECK_NOTHROW(optimizer.evalNextBestControl(ps, twist, path));

#ifdef DO_BENCHMARKS
    WARN("Path with " << poses_count);
    BENCHMARK("evalNextControl Benchmark") {
      return optimizer.evalNextBestControl(ps, twist, path);
    };
#endif
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(st);
  costmap_ros.reset();
}

TEST_CASE("Optimizer evaluates Trajectory From Control Sequence", "[collision]") {
  using T = float;
  std::string node_name = "TestNode";
  std::string costmap_node_name = "cost_map_node";

  std::vector<rclcpp::Parameter> params_;
  rclcpp::NodeOptions options;
  setUpOptimizerParams(params_);
  options.parameter_overrides(params_);

  auto & model = mppi::models::NaiveModel<T>;
  auto optimizer = mppi::optimization::Optimizer<T>();
  auto state = rclcpp_lifecycle::State{};
  auto grid_map = std::make_shared<grid_map::GridMap>();

  // params for gridmap
  std::string layer_name = "elevation";
  std::string frame = "odom";
  float grid_map_lenght_x = 2;
  float grid_map_lenght_y = 2;
  float grid_map_resolution = 0.05;
  float grid_map_pose_x = 0.0;
  float grid_map_pose_y = 0.0;
  
  grid_map->add(layer_name);
  grid_map->setFrameId(frame);
  grid_map->setGeometry(
    grid_map::Length(grid_map_lenght_x, grid_map_lenght_y),
    grid_map_resolution,
    grid_map::Position(grid_map_pose_x, grid_map_pose_y)
  );  
  
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_node_name);
  costmap_ros->on_configure(state);

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
  optimizer.on_configure(node, node_name, costmap_ros, grid_map, model);
  optimizer.on_activate();

  size_t reference_path_lenght = GENERATE(50);  
  float start_point_x = GENERATE(1.25, 0.7);
  float start_point_y = GENERATE(1.5, 0.5);

  SECTION("Optimizer produces a trajectory that does not cross obstacles on the gridmap") {

    auto time = node->get_clock()->now();
    nav_msgs::msg::Path reference_path;
    geometry_msgs::msg::PoseStamped reference_goal_pose;
    geometry_msgs::msg::PoseStamped init_robot_pose;         
    geometry_msgs::msg::Twist init_robot_vel; 
    float robot_clearance = 0.1;          
    float x_step = -0.012;
    float y_step = -0.002;
    init_robot_pose.pose.position.x = start_point_x;
    init_robot_pose.pose.position.y = start_point_y;

    // params for ramp on the gridmap
    float ramp_height = 0.6;
    float ramp_step = 0.15;
    float ramp_size = 0.4;
    grid_map::Index ramp_left_upper_corner_cells(8, 3);

    // params for hill on the grid map
    float max_hill_height = 0.36;
    float hill_step = 0.03;
    float hill_size = 0.8;
    grid_map::Index hill_left_upper_corner_cells(6, 16);

    // lambda expression for setting header
    auto setHeader = [&](auto &&msg) {
      msg.header.frame_id = frame;
      msg.header.stamp = time;
    };

    // lambda expression for refernce path generation
    auto fillRealPath = [&](size_t count) {
      for (size_t i = 0; i < count; i++) {
          reference_goal_pose.pose.position.x = i*x_step + start_point_x;
          reference_goal_pose.pose.position.y = i*y_step + start_point_y;
          reference_path.poses.push_back(reference_goal_pose);
      }
    };  

    setHeader(reference_goal_pose);
    setHeader(init_robot_pose);
    setHeader(reference_path);
    fillRealPath(reference_path_lenght);

    grid_map->get(layer_name).setZero();
    addHillToGridMapLayer(
      *grid_map, 
      layer_name, 
      max_hill_height, 
      hill_step, 
      hill_size,
      hill_left_upper_corner_cells
    );

    addRampToGridMapLayer(
      *grid_map,
      layer_name,
      ramp_height,
      ramp_step,
      ramp_size,
      ramp_left_upper_corner_cells
    );

    // update controal sequence in optimizer
    CHECK_NOTHROW(optimizer.evalNextBestControl(init_robot_pose, init_robot_vel, reference_path));
    // get best trajectory from optimizer
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(init_robot_pose, init_robot_vel);

#ifdef TEST_DEBUG_INFO
    printGridMapLayerWithTrajectoryAndGoal(*grid_map, layer_name, trajectory, reference_goal_pose);
#endif      
    CHECK(!InCollison(*grid_map, layer_name, robot_clearance, trajectory));
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(state);
  costmap_ros.reset();
}