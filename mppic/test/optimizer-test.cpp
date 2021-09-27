#include <cstdio>
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
  printf("GridMap \n start point=-100 trajectory=-1 goal point=100 \n");
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
* Fills the entire layer with zero values.
* @param grid_map grid map to be updated.
* @param layer_name name of the map layer.
*/
void makeFlatGridMapLayer(grid_map::GridMap & grid_map, std::string & layer_name){
  for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
    grid_map.at(layer_name, *it) = 0.0;
  }
}


/*!
* Adds a hill to the grid map
* @param grid_map grid map to be updated.
* @param layer_name name of the map layer.
* @param hill_height hill height in meters.
* @param hill_step step of changing the height between cells in meters.
* @param hill_size_m size (length) of the hill in meters.
* @param cx the x coordinate of the upper left corner of the hill.
* @param cy the y coordinate of the upper left corner of the hill.
*/
void addHillToGridMapLayer(grid_map::GridMap & grid_map, std::string layer_name,
                          const float & hill_height,
                          const float & hill_step, const float & hill_size_m,
                          const int & cx, const int & cy){
  
  double grid_map_resolution = grid_map.getResolution();                    
  float hill_size_cells = hill_size_m / grid_map_resolution;
  auto & layer_matrix = grid_map.get(layer_name);
  float h = hill_step;
  float size_x = hill_size_cells;
  float size_y = hill_size_cells;
  int cx_ = cx;
  for (size_t j = cy; j <= cy+hill_size_cells/2; j++){
    layer_matrix.block(cx_, j, size_x, size_y).setConstant(h);
    cx_ = cx_ + 1;
    size_x = size_x - 2;
    size_y = size_y - 2;
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
* @param cx the x coordinate of the upper left corner of the ramp.
* @param cy the y coordinate of the upper left corner of the ramp.
*/
void addRampToGridMapLayer(grid_map::GridMap & grid_map, std::string layer_name,
                         const float & ramp_height,
                         const float & ramp_step, const float & ramp_size,
                         const int & cx, const int & cy){

  double grid_map_resolution = grid_map.getResolution();
  float size = ramp_size / grid_map_resolution;
  auto & layer_matrix = grid_map.get(layer_name);
  float h = ramp_step;

  for (size_t j = cy; j < cy + size; j++){
    for (size_t i = cx; i < cx + size; i++){
      layer_matrix.row(j)(i) = h;
    }
    if (h + ramp_step <= ramp_height){
      h = h + ramp_step;
    }
  }
}

/*!
* Prints map, trajectory and goal in to cout.
* @param grid_map map to be printed.
* @param layer_name name of the map layer.
* @param robot_clearance the maximum height between the cells of the grid map that the robot can overcome
* @param trajectory trajectory to be printed.
*/
bool checkTrajectoryCollision(const grid_map::GridMap & grid_map, 
                              const std::string & layer_name,
                              const float & robot_clearance,
                              const auto & trajectory){

  auto & matrix = grid_map.get(layer_name);
  double grid_map_resolution = grid_map.getResolution();
  float origin_x = grid_map.getPosition()(0);
  float origin_y = grid_map.getPosition()(1);
  // std::cout<<robot_clearance<<std::endl;
  int traj_point_x_prev = (trajectory(0, 0) - origin_x)/ grid_map_resolution;
  int traj_point_y_prev = (trajectory(0, 1) - origin_y)/ grid_map_resolution;
  for (size_t i = 1; i < trajectory.shape()[0]; ++i){
    int traj_point_x = (trajectory(i, 0) - origin_x)/ grid_map_resolution;
    int traj_point_y = (trajectory(i, 1) - origin_y)/ grid_map_resolution;
    bool can_drive = abs(matrix(traj_point_x, traj_point_y) - matrix(traj_point_x_prev, traj_point_y_prev)) > robot_clearance;
    if (can_drive){
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

  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name);
  optimizer.on_configure(node, node_name, costmap_ros, grid_map, model);
  optimizer.on_activate();

  size_t reference_path_lenght = GENERATE(50);  
  // int obstacle_step_k = GENERATE(1); 
  float start_point_x = GENERATE(1.25, 0.7);
  float start_point_y = GENERATE(1.5, 0.5);

  SECTION("Optimizer produces a trajectory that does not cross obstacles on the gridmap") {

    auto time = node->get_clock()->now();
    // float start_point_x = 1.25;
    // float start_point_y = 1.25;
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
    float ramp_left_upper_corner_cells_x = 8;
    float ramp_left_upper_corner_cells_y = 3;

    // params for hill on the grid map
    float max_hill_height = 0.36;
    float hill_step = 0.03;
    float hill_size = 0.8;
    int hill_left_upper_corner_cells_x = 6;
    int hill_left_upper_corner_cells_y = 16;

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
          // std::cout<<reference_goal_pose.pose.position.x<< " "<<reference_goal_pose.pose.position.y<<std::endl;
          reference_path.poses.push_back(reference_goal_pose);
      }
    };  

    setHeader(reference_goal_pose);
    setHeader(init_robot_pose);
    setHeader(reference_path);
    fillRealPath(reference_path_lenght);

    makeFlatGridMapLayer(*grid_map, layer_name);
    addHillToGridMapLayer(
      *grid_map, 
      layer_name, 
      max_hill_height, 
      hill_step, 
      hill_size,
      hill_left_upper_corner_cells_x, 
      hill_left_upper_corner_cells_y
    );

    addRampToGridMapLayer(
      *grid_map,
      layer_name,
      ramp_height,
      ramp_step,
      ramp_size,
      ramp_left_upper_corner_cells_x, 
      ramp_left_upper_corner_cells_y
    );

    // update controal sequence in optimizer
    CHECK_NOTHROW(optimizer.evalNextBestControl(init_robot_pose, init_robot_vel, reference_path));
    // get best trajectory from optimizer
    auto trajectory = optimizer.evalTrajectoryFromControlSequence(init_robot_pose, init_robot_vel);
    // check trajectory for collision
    bool result = checkTrajectoryCollision(*grid_map, layer_name, robot_clearance, trajectory);
    if (result == true){
      printGridMapLayerWithTrajectoryAndGoal(*grid_map, layer_name, trajectory, reference_goal_pose);
    }
    REQUIRE(result == false);
  }

  optimizer.on_deactivate();
  optimizer.on_cleanup();
  costmap_ros->on_cleanup(state);
  costmap_ros.reset();
}