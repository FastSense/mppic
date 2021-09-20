#ifdef DO_BENCHMARKS
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#endif

#include <catch2/catch.hpp>

#include "nav2_util/lifecycle_node.hpp"

#include "mppi/GridMapHandler.hpp"

#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

TEST_CASE("Handler must get Grid Map", "") {

  std::string node_name = "TestNode";
  std::string grid_map_topic = "elevation_map";
  auto inner_grid = std::make_shared<grid_map::GridMap>();

  auto grid_map_handler = mppi::handlers::GridMapHandler();

  auto &&node = [&] {
    std::vector<rclcpp::Parameter> params;
    rclcpp::NodeOptions options;
    params.push_back(rclcpp::Parameter("grid_map_topic", grid_map_topic));
    options.parameter_overrides(params);
    return std::make_shared<nav2_util::LifecycleNode>(node_name, "", false,
                                                      options);
  }();

  auto grid_publisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
      grid_map_topic, rclcpp::QoS(1).transient_local());

  grid_publisher->on_activate();
  grid_map_handler.on_configure(node, node_name, inner_grid);
  grid_map_handler.on_activate();

  auto state = rclcpp_lifecycle::State{};

  std::string layer_name = "elevation";
  SECTION("Check if we got GridMap") {
    auto &&grid_to_send = [&]() {
      auto grid = grid_map::GridMap({layer_name});
      grid.setFrameId("odom");
      grid.setGeometry(grid_map::Length(1.2, 2.0), 0.05,
                       grid_map::Position(0.0, -0.1));

      for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
        grid.at(layer_name, *it) = 2.0;
      }
      return grid;
    }();

    RCLCPP_INFO(
        node->get_logger(),
        "Created grid with size %f x %f m (%i x %i cells).\n"
        " The center of the map is located at (%f, %f) in the %s frame.",
        grid_to_send.getLength().x(), grid_to_send.getLength().y(),
        grid_to_send.getSize()(0), grid_to_send.getSize()(1),
        grid_to_send.getPosition().x(), grid_to_send.getPosition().y(),
        grid_to_send.getFrameId().c_str());
    grid_to_send.setTimestamp(node->get_clock()->now().nanoseconds());

    auto publish_grid = [&](const grid_map::GridMap &outputMap) {
      std::unique_ptr<grid_map_msgs::msg::GridMap> outputMessage;
      outputMessage = grid_map::GridMapRosConverter::toMessage(outputMap);
      grid_publisher->publish(std::move(outputMessage));
    };

    RCLCPP_INFO(node->get_logger(), "Publishing Grid Map...");
    publish_grid(grid_to_send);
    RCLCPP_INFO(node->get_logger(), "Grid Map Published");

    rclcpp::spin_some(node->get_node_base_interface());

    CHECK(inner_grid->at(layer_name, grid_map::Index{5, 5}) == 2.0);
    CHECK(inner_grid->at(layer_name, grid_map::Index{2, 2}) == 2.0);
  }

  grid_map_handler.on_deactivate();
  grid_map_handler.on_cleanup();
}
