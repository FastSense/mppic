

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
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

class RosLockGuard
{
public:
	RosLockGuard() { rclcpp::init(0, nullptr); }
	~RosLockGuard() { rclcpp::shutdown(); }
};
RosLockGuard g_rclcpp;

constexpr int default_iteration_count = 5;
constexpr double default_lookahead_dist = 5.0;
constexpr int default_time_steps = 50;
constexpr double default_model_dt = 0.1;
constexpr int default_batch_size = 200;
constexpr double default_slope_cost_power = 1.0;
constexpr double default_slope_cost_weight = 10.0;
constexpr double default_slope_crit = 0.5;
constexpr double default_roughness_cost_power = 1.0;
constexpr double default_roughness_cost_weight = 10.0;
constexpr double default_roughness_crit = 0.5;

constexpr bool default_use_slope_traversability = false;
constexpr double default_slope_traversability_delta = 0.1;
constexpr double default_slope_traversability_alpha = 0.5;
constexpr double default_slope_traversability_lambda = 1.0;
constexpr double default_slope_traversability_cost_power = 1.0;
constexpr double default_slope_traversability_cost_weight = 10.0;
constexpr double default_inflation_radius = 0.3;

void read_parameters(std::string config_file, std::vector<rclcpp::Parameter> &params, double &step_dt, double &robot_size, size_t &steps, double &delta)
{
	std::unordered_map<std::string, rclcpp::ParameterValue> params_dict = {};
	tinyxml2::XMLDocument xml_config;

	if (xml_config.LoadFile(config_file.c_str()) == tinyxml2::XMLError::XML_SUCCESS)
	{
		std::cout << "Config file found\n";

		rclcpp::ParameterValue param_value_ros;

		tinyxml2::XMLElement *root = xml_config.RootElement();
		if (root)
		{
			for (tinyxml2::XMLElement *e = root->FirstChildElement(); e != NULL; e = e->NextSiblingElement())
			{
				std::stringstream converter;
				std::string param_name = e->Value();

				if (param_name == "step_dt")
				{
					step_dt = e->FloatAttribute("value", 0.0);
					continue;
				}
				else if (param_name == "steps")
				{
					steps = e->Int64Attribute("value", 0);
					continue;
				}
				else if (param_name == "goal_delta")
				{
					delta = e->FloatAttribute("value", 0);
					continue;
				}
				else if (param_name == "footprint_radius")
				{
					robot_size = e->FloatAttribute("value", 0.0);
				}

				std::string param_type = e->Attribute("type");

				if (param_type == "int64")
				{
					int param_value = e->Int64Attribute("value", 0);
					param_value_ros = rclcpp::ParameterValue(param_value);
				}
				else if (param_type == "float64")
				{
					double param_value = e->FloatAttribute("value", 0.0);
					param_value_ros = rclcpp::ParameterValue(param_value);
				}
				else if (param_type == "bool")
				{
					bool param_value = e->BoolAttribute("value", false);
					param_value_ros = rclcpp::ParameterValue(param_value);
				}
				else if (param_type == "string")
				{
					std::string param_value = e->Attribute("value");
					param_value_ros = rclcpp::ParameterValue(param_value);
				}
				else
				{
					std::string param_value = e->Attribute("value");
					param_value_ros = rclcpp::ParameterValue(param_value);
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

	for (const std::pair<std::string, rclcpp::ParameterValue> &param : params_dict)
	{
		params.push_back(rclcpp::Parameter("TestNode." + param.first, param.second));
	}
}


void read_map(std::string map_file, grid_map::GridMap &grid_map)
{
	std::string layer_name = "elevation";
	std::string frame = "odom";
	grid_map.add(layer_name);
	grid_map.setFrameId(frame);

	tinyxml2::XMLDocument xml_map;
	if (xml_map.LoadFile(map_file.c_str()) == tinyxml2::XMLError::XML_SUCCESS)
	{
		std::cout << "Map file found\n";
		tinyxml2::XMLElement *root = xml_map.RootElement();
		if (root)
		{
			double resolution = root->FirstChildElement("resolution")->FloatText();
			size_t width = root->FirstChildElement("width")->IntText();
			size_t height = root->FirstChildElement("height")->IntText();

			float lenght_x = height * resolution;
			float lenght_y = width * resolution;
			float center_x = 0.0;
			float center_y = 0.0;
			grid_map.setGeometry(
				grid_map::Length(lenght_x, lenght_y),
				resolution,
				grid_map::Position(center_x, center_y));
			auto &layer_matrix = grid_map.get(layer_name);

			auto grid = root->FirstChildElement("grid");
			size_t row_count = 0;
			for (tinyxml2::XMLElement *e = grid->FirstChildElement(); e != NULL && row_count < height; e = e->NextSiblingElement(), row_count++)
			{
				std::string row_str = e->GetText();
				std::stringstream converter;
				converter << row_str;
				for (size_t col_count = 0; col_count < width; col_count++)
				{
					float z;
					converter >> z;
					layer_matrix(row_count, col_count) = z;
				}
				std::cout << std::endl;
			}
			std::cout << "(" << lenght_x << ", " << lenght_y << ") map read successfully\n";
		}
		else
		{
			std::cout << "Cannot find root element in map file\n";
			exit(-1);
		}
	}
	else
	{
		std::cout << "Cannot open map file\n";
		exit(-1);
	}
}


void read_tasks(std::string tasks_file, std::vector<geometry_msgs::msg::Pose> &starts, std::vector<geometry_msgs::msg::Pose> &goals)
{
	tinyxml2::XMLDocument xml_tasks;
	size_t task_count = 0;
	starts.clear();
	goals.clear();
	if (xml_tasks.LoadFile(tasks_file.c_str()) == tinyxml2::XMLError::XML_SUCCESS)
	{
		std::cout << "Tasks file found\n";
		tinyxml2::XMLElement *root = xml_tasks.RootElement();
		if (root)
		{
			std::stringstream converter;
			for (tinyxml2::XMLElement *e = root->FirstChildElement(); e != NULL; e = e->NextSiblingElement())
			{
				geometry_msgs::msg::Pose start, goal;
				tinyxml2::XMLElement *start_elem = e->FirstChildElement("start");
				start.position.x = start_elem->FloatAttribute("pos_x");
				start.position.y = start_elem->FloatAttribute("pos_y");
				start.position.z = 0.0;
				start.orientation.x = start_elem->FloatAttribute("q_x");
				start.orientation.y = start_elem->FloatAttribute("q_y");
				start.orientation.z = start_elem->FloatAttribute("q_z");
				start.orientation.w = start_elem->FloatAttribute("q_w");

				tinyxml2::XMLElement *goal_elem = e->FirstChildElement("goal");
				goal.position.x = goal_elem->FloatAttribute("pos_x");
				goal.position.y = goal_elem->FloatAttribute("pos_y");
				goal.position.z = 0.0;
				goal.orientation.x = goal_elem->FloatAttribute("q_x");
				goal.orientation.y = goal_elem->FloatAttribute("q_y");
				goal.orientation.z = goal_elem->FloatAttribute("q_z");
				goal.orientation.w = goal_elem->FloatAttribute("q_w");

				starts.push_back(start);
				goals.push_back(goal);
				task_count++;
			}
			std::cout << task_count << " tasks read successfully\n";
		}
		else
		{
			std::cout << "Cannot find root element in tasks file\n";
			exit(-1);
		}
	}
	else
	{
		std::cout << "Cannot open tasks file\n";
		exit(-1);
	}
}


void fill_reference_path(geometry_msgs::msg::Pose &start, geometry_msgs::msg::Pose &goal, geometry_msgs::msg::PoseStamped path_pose_template, nav_msgs::msg::Path &reference_path)
{
	double interpolation_resolution_ = 0.1;

	int total_number_of_loop = std::hypot(goal.position.x - start.position.x, goal.position.y - start.position.y) / interpolation_resolution_;
	double x_increment = (goal.position.x - start.position.x) / total_number_of_loop;
	double y_increment = (goal.position.y - start.position.y) / total_number_of_loop;
	reference_path.poses.clear();

	for (int i = 0; i < total_number_of_loop; i++)
	{
		path_pose_template.pose.position.x = start.position.x + x_increment * i;
		path_pose_template.pose.position.y = start.position.y + y_increment * i;
		path_pose_template.pose.position.z = 0.0;
		path_pose_template.pose.orientation.x = 0.0;
		path_pose_template.pose.orientation.y = 0.0;
		path_pose_template.pose.orientation.z = 0.0;
		path_pose_template.pose.orientation.w = 1.0;
		reference_path.poses.push_back(path_pose_template);
	}

	path_pose_template.pose = goal;
	reference_path.poses.push_back(path_pose_template);
}


void print_grid_map(grid_map::GridMap &grid_map, const std::string &layer_name)
{
	std::cout << grid_map.get(layer_name) << std::endl;
}


int main(int argc, char *argv[])
{
	// Preparations and parameter reading

	if (argc < 5)
	{
		std::cout << "Not enough parameters\n";
		return 0;
	}

	using T = float;
	std::string frame = "odom";

	std::string config_file = argv[1];
	std::string map_file = argv[2];
	std::string tasks_file = argv[3];
	std::string prefix_res = argv[4];

	std::string costmap_node_name = "cost_map_node";
	std::string node_name = "TestNode";

	std::vector<rclcpp::Parameter> params;
	rclcpp::NodeOptions options;
	double step_dt;
	double robot_size;
	double delta;
	size_t steps;

	read_parameters(config_file, params, step_dt, robot_size, steps, delta);
	options.parameter_overrides(params);

	// map reading and creation
	auto grid_map = std::make_shared<grid_map::GridMap>();
	read_map(map_file, *grid_map);

	// reading tasks and configuring mppi
	std::vector<geometry_msgs::msg::Pose> starts;
	std::vector<geometry_msgs::msg::Pose> goals;

	read_tasks(tasks_file, starts, goals);
	size_t number_of_tasks = starts.size();

	std::cout << "Start\n";
	for (size_t task = 0; task < number_of_tasks; task++)
	{

		std::ofstream myfile;
		std::string res_file_name = prefix_res + "_result_task_" + std::to_string(task) + ".txt";
		myfile.open(res_file_name);

		std::cout << "Task " << task << "\n";

		auto state = rclcpp_lifecycle::State{};

		auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(costmap_node_name);

		costmap_ros->on_configure(state);
		costmap_ros->set_parameter(rclcpp::Parameter("track_unknown_space", rclcpp::ParameterValue(false)));
		costmap_ros->set_parameter(rclcpp::Parameter("unknown_cost_value", rclcpp::ParameterValue(0)));

		auto &model = mppi::models::NaiveModel<T>;
		auto optimizer = mppi::optimization::Optimizer<T>();

		auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(node_name, options);

		optimizer.on_configure(node, node_name, costmap_ros, grid_map, model);

		optimizer.on_activate();

		auto time = node->get_clock()->now();
		auto setHeader = [&](auto &&msg)
		{
			msg.header.frame_id = frame;
			msg.header.stamp = time;
		};

		nav_msgs::msg::Path reference_path;
		setHeader(reference_path);
		geometry_msgs::msg::PoseStamped pose_template, init_robot_pose;
		init_robot_pose.pose = starts[task];
		setHeader(pose_template);
		setHeader(init_robot_pose);

		fill_reference_path(starts[task], goals[task], pose_template, reference_path);
		geometry_msgs::msg::Twist init_robot_vel;

		double dist_to_goal;
		double time_sum = 0.0;
		size_t step;
		for (step = 0; step < steps; step++)
		{

			std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

			auto control = optimizer.evalNextBestControl(init_robot_pose, init_robot_vel, reference_path);

			std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
			time_sum += static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) / 1e6;

			float v = control.twist.linear.x;
			float w = control.twist.angular.z;

			float yaw = w * step_dt;
			yaw += tf2::getYaw(init_robot_pose.pose.orientation);
			float v_x = v * std::cos(yaw);
			float v_y = v * std::sin(yaw);

			float x = init_robot_pose.pose.position.x + v_x * step_dt;
			float y = init_robot_pose.pose.position.y + v_y * step_dt;

			myfile << x << " " << y << " " << yaw << "\n";
			dist_to_goal = std::hypot(goals[task].position.x - x, goals[task].position.y - y);

			if (dist_to_goal < delta)
			{
				myfile << "True\n";
				break;
			}

			init_robot_vel = control.twist;
			init_robot_pose.pose.position.x = x;
			init_robot_pose.pose.position.y = y;
			init_robot_pose.pose.position.z = 0.0;

			tf2::Quaternion orient;
			orient.setRPY(0, 0, yaw);
			orient.normalize();
			init_robot_pose.pose.orientation = tf2::toMsg(orient);
			fill_reference_path(init_robot_pose.pose, goals[task], pose_template, reference_path);

		}
		double time_avg = time_sum / step;

		std::cout << "End task\n";
		if (not(dist_to_goal < delta))
		{
			myfile << "False\n";
		}

		myfile << time_avg << "\n"; 

		optimizer.on_deactivate();
		optimizer.on_cleanup();
		costmap_ros->on_cleanup(state);
		costmap_ros.reset();
		myfile.close();
	}
	exit(0);
	return 0;
}