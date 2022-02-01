#include <iostream>
#include <string>
#include <cmath>
#include <unordered_map>
#include <experimental/filesystem>
#include <sstream>

#include <tinyxml2.h>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include <rclcpp/executors.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "mppi/Models.hpp"
#include "mppi/impl/Optimizer.hpp"




constexpr int       default_iteration_count = 5;
constexpr double    default_lookahead_dist = 5.0;
constexpr int       default_time_steps = 50;
constexpr double    default_model_dt = 0.1;
constexpr int       default_batch_size = 200;
constexpr double    default_slope_cost_power = 1.0;
constexpr double    default_slope_cost_weight = 10.0;
constexpr double    default_slope_crit =  0.5;
constexpr double    default_roughness_cost_power = 1.0;
constexpr double    default_roughness_cost_weight = 10.0;
constexpr double    default_roughness_crit = 0.5;

constexpr bool      default_use_slope_traversability = false;
constexpr double    default_slope_traversability_delta = 0.1;
constexpr double    default_slope_traversability_alpha = 0.5;
constexpr double    default_slope_traversability_lambda = 1.0;
constexpr double    default_slope_traversability_cost_power = 1.0;
constexpr double    default_slope_traversability_cost_weight = 10.0;
constexpr double    default_inflation_radius = 0.3;



void read_parameters(std::string config_file, std::vector<rclcpp::Parameter> &params, double &step_dt, double &robot_size, size_t &steps)
{
    // TODO: Add all parameters of mppi
    // TODO: Add experiment parameters
  std::unordered_map<std::string, rclcpp::ParameterValue> params_dict = 
  {
    {"iteration_count",                   rclcpp::ParameterValue(default_iteration_count)},
    {"lookahead_dist",                    rclcpp::ParameterValue(default_lookahead_dist)},
    {"time_steps",                        rclcpp::ParameterValue(default_time_steps)},
    {"model_dt",                          rclcpp::ParameterValue(default_model_dt)}
    {"batch_size",                        rclcpp::ParameterValue(default_batch_size)},
    {"slope_cost_power",                  rclcpp::ParameterValue(default_slope_cost_power)},
    {"slope_cost_weight",                 rclcpp::ParameterValue(default_slope_cost_weight)},
    {"slope_crit",                        rclcpp::ParameterValue(default_slope_crit)},
    {"roughness_cost_power",              rclcpp::ParameterValue(default_roughness_cost_power)},
    {"roughness_cost_weight",             rclcpp::ParameterValue(default_roughness_cost_weight)},
    {"roughness_crit",                    rclcpp::ParameterValue(default_roughness_crit)},


    {"slope_traversability_delta",        rclcpp::ParameterValue(default_slope_traversability_delta)},
    {"slope_traversability_alpha",        rclcpp::ParameterValue(default_slope_traversability_alpha)},
    {"slope_traversability_lambda",       rclcpp::ParameterValue(default_slope_traversability_lambda)},
    {"slope_traversability_cost_power",   rclcpp::ParameterValue(default_slope_traversability_cost_power)},
    {"slope_traversability_cost_weight",  rclcpp::ParameterValue(default_slope_traversability_cost_weight)},
    {"inflation_radius",                  rclcpp::ParameterValue(default_inflation_radius)}
  };


  tinyxml2::XMLDocument xml_config;

  if (xml_config.LoadFile(config_file.c_str()) == tinyxml2::XMLError::XML_SUCCESS)
  {
    std::cout << "Config file found\n";
    
    rclcpp::ParameterValue param_value_ros;

    tinyxml2::XMLElement * root = xml_config.RootElement();
    if(root)
    {
      for(tinyxml2::XMLElement* e = root->FirstChildElement(); e != NULL; e = e->NextSiblingElement())
      {   
          std::stringstream converter;
          std::string param_name = e->Value();
          std::string param_type = e->Attribute("type");
          std::string param_value_str = e->Attribute("value");
          if(param_type == "int64")
          {
            int param_value;
            converter << param_value_str;
            converter >> param_value;
            param_value_ros = rclcpp::ParameterValue(param_value);
          }
          else if(param_type == "float64")
          {
            double param_value;            
            converter << param_value_str;
            converter >> param_value;
            param_value_ros = rclcpp::ParameterValue(param_value);
          }
          else if(param_type == "bool")
          {
            bool param_value;
            converter << param_value_str;
            converter >> param_value;
            param_value_ros = rclcpp::ParameterValue(param_value);
          }
          else
          {
            param_value_ros = rclcpp::ParameterValue(param_value_str);
          }
        params_dict[param_name] = param_value_ros;
      }
        std::cout << "Parameters read successfully\n";
    }
    else
    {
       std::cout << "Cannot find root element in file. Default parameters will be used\n";
    }
  }
  else
  {
    std::cout << "Cannot open config file. Default parameters will be used\n";
  }

  for( const std::pair<std::string, rclcpp::ParameterValue>& param : params_dict) 
  {
    params.push_back(rclcpp::Parameter("TestNode." + param.first, param.second));
  }
}

// TODO
void read_map(std::string config_file, grid_map::GridMap & grid_map)
{
    // std::string layer_name = "elevation";
    // std::string frame = "odom";
    
    // float grid_map_lenght_x = 2;
    // float grid_map_lenght_y = 2;
    // float grid_map_resolution = 0.05;
    // float grid_map_pose_x = 0.0;
    // float grid_map_pose_y = 0.0;

    // grid_map->add(layer_name);
    // grid_map->setFrameId(frame);
    // grid_map->setGeometry(
    // grid_map::Length(grid_map_lenght_x, grid_map_lenght_y),
    // grid_map_resolution,
    // grid_map::Position(grid_map_pose_x, grid_map_pose_y)
    // );  
}

// TODO
void read_tasks(std::string tasks_file, std::vector<geometry_msgs::msg::Pose> &starts, std::vector<geometry_msgs::msg::Pose> &goals)
{

}

// TODO
void fill_reference_path(geometry_msgs::msg::Pose &start, geometry_msgs::msg::Pose &goal, nav_msgs::msg::Path &reference_path)
{

        // lambda expression for refernce path generation
    // auto fillRealPath = [&](size_t count) {
    // for (size_t i = 0; i < count; i++) {
    // reference_goal_pose.pose.position.x = i * x_step + start_point_x;
    // reference_goal_pose.pose.position.y = i * y_step + start_point_y;
    // reference_path.poses.push_back(reference_goal_pose);
    // }
    // };  
    
}


int main(int argc, char *argv[])
{
    // Preparations and parameter reading

    if(argc < 4)
    {   
        std::cout << "Not enough parameters\n";
        return 0;
    }

    using T = float;
    std::string frame = "odom";

    std::string config_file = argv[1];
    std::string map_file = argv[2];  
    std::string tasks_file = argv[3];  

    std::string costmap_node_name = "cost_map_node";
            std::string node_name = "TestNode";
    

    std::vector<rclcpp::Parameter> params;
    rclcpp::NodeOptions options;
    float step_dt;
    float robot_size;
    size_t steps;
    
    read_parameters(config_file, params, step_dt, robot_size, steps);
    options.parameter_overrides(params);

    
    
    // map reading and creation
    auto grid_map = std::make_shared<grid_map::GridMap>();
    read_map(map_file, grid_map);


    // reading tasks and configuring mppi 
    std::vector<geometry_msgs::msg::Pose> starts;
    std::vector<geometry_msgs::msg::Pose> goals;
    
    read_tasks(tasks_file, starts, goals);
    size_t number_of_tasks = starts.size();




    for(size_t task = 0; task < number_of_tasks; task++)
    {

        auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_node_name);
        auto state = rclcpp_lifecycle::State{};
        costmap_ros->on_configure(state);
        costmap_ros->set_parameter(rclcpp::Parameter("track_unknown_space", rclcpp::ParameterValue(false)));
        costmap_ros->set_parameter(rclcpp::Parameter("unknown_cost_value", rclcpp::ParameterValue(0)));
        auto & model = mppi::models::NaiveModel<T>;
        auto optimizer = mppi::optimization::Optimizer<T>();

        auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);
        optimizer.on_configure(node, node_name, costmap_ros, grid_map, model);
        optimizer.on_activate();
        auto time = node->get_clock()->now();
        void setHeader(auto &&msg, ) 
        {
            msg.header.frame_id = frame;
            msg.header.stamp = time;
        };
    
        nav_msgs::msg::Path reference_path;
        setHeader(reference_path);
        fill_reference_path(starts[i], goals[i], reference_path);

        geometry_msgs::msg::PoseStamped init_robot_pose;
        init_robot_pose.pose = starts[i];
        setHeader(init_robot_pose); 

        geometry_msgs::msg::Twist init_robot_vel; 

        for(size_t step = 0; step < steps; step++)
        {
            auto control = optimizer.evalNextBestControl(init_robot_pose, init_robot_vel, reference_path);
            float v = control.twist.linear.x;
            float w = control.twist.angular.z;
  
            float yaw = w * step_dt;
            yaw += tf2::getYaw(init_robot_pose.pose.orientation);
            float v_x = v * std::cos(yaw);
            float v_y = v * std::sin(yaw);

            float x = init_robot_pose.pose.position.x + v_x * step_dt;
            float y = init_robot_pose.pose.position.y + v_y * step_dt;

            std::cout << x << " " << y << " " << yaw << "\n";
            // TODO: Update path, pose and velocity for next step
            // TODO: Save result to txt file
        }

        optimizer.on_deactivate();
        optimizer.on_cleanup();
        costmap_ros->on_cleanup(state);
        costmap_ros.reset();
    }

    return 0;
}