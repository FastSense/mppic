#include "utils/common.hpp"
#include "utils/geometry.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

#include "mppi/GridMapHandler.hpp"

namespace mppi::handlers
{

void GridMapHandler::on_configure()
{
  getParams();
  createSubscribers();
  RCLCPP_INFO(logger_, "Configured");

}
void GridMapHandler::on_cleanup() {}
void GridMapHandler::on_activate() {}
void GridMapHandler::on_deactivate() {}


void GridMapHandler::getParams()
{
  auto getParam = [&](const std::string & param_name, auto default_value) {
      std::string name = node_name_ + '.' + param_name;
      return utils::getParam(name, default_value, parent_);
    };

  (void) getParam;
}

void GridMapHandler::createSubscribers()
{
  subscriber_ = parent_->create_subscription<grid_map_msgs::msg::GridMap>(
    input_topic_, 1, std::bind(&GridMapHandler::gridMapCallback, this, std::placeholders::_1) );
}

void GridMapHandler::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr message)
{
  grid_map::GridMapRosConverter::fromMessage(*message, *grid_map_);
}


} // namespace mppi::handlers
