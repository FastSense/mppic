#pragma once

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

namespace mppi::handlers
{

class GridMapHandler
{

public:
  GridMapHandler() = default;
  ~GridMapHandler() = default;

  GridMapHandler(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & parent,
    const std::string & node_name,
    const std::string & input_topic,
    const std::shared_ptr<grid_map::GridMap> & grid_map,
    const std::shared_ptr<tf2_ros::Buffer> & buffer)
  : parent_(parent), node_name_(node_name), grid_map_(grid_map), input_topic_(input_topic), tf_buffer_(buffer) {}

  void on_configure();
  void on_cleanup();
  void on_activate();
  void on_deactivate();


private:
  void getParams();
  void createSubscribers();

  void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)


private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_;
  std::string node_name_;
  std::string input_topic_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<grid_map::GridMap> grid_map_;

  rclcpp::Logger logger_{rclcpp::get_logger("MPPI GridMapHandler")};
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscriber_;
};

} // namespace mppi::handlers
