#pragma once

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"

#include "mppi/Optimizer.hpp"

#include "xtensor/xmath.hpp"
#include "xtensor/xrandom.hpp"
#include "xtensor/xadapt.hpp"

#include "xtensor-blas/xlinalg.hpp"

#include "utils/common.hpp"
#include "utils/geometry.hpp"

#include <limits>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <cmath>
#include <array>

namespace mppi::optimization
{

	template <typename T, typename Model>
	auto Optimizer<T, Model>::evalNextBestControl(
		const geometry_msgs::msg::PoseStamped &robot_pose,
		const geometry_msgs::msg::Twist &robot_speed,
		const nav_msgs::msg::Path &plan)
		-> geometry_msgs::msg::TwistStamped
	{
		auto t1 = std::chrono::high_resolution_clock::now();
		for (int i = 0; i < iteration_count_; ++i)
		{
			generated_trajectories_ = generateNoisedTrajectories(robot_pose, robot_speed);
			auto costs = evalBatchesCosts(generated_trajectories_, plan, robot_pose);
			updateControlSequence(costs);
		}
		auto result = getControlFromSequence(plan.header.stamp, costmap_ros_->getBaseFrameID());

		shiftControlSequence();

		auto t2 = std::chrono::high_resolution_clock::now();
		auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
		double time_for_eval = dt.count();
		std_msgs::msg::Float64 time_msg;
		time_msg.data = time_for_eval;
		eval_time_publisher_->publish(time_msg);

		return result;
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::on_configure(
		const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &parent,
		const std::string &node_name,
		const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros,
		const std::shared_ptr<grid_map::GridMap> &grid_map,
		Model &&model)
		-> void
	{
		parent_ = parent;
		node_name_ = node_name;
		costmap_ros_ = costmap_ros;
		model_ = model;
		grid_map_ = grid_map;

		costmap_ = costmap_ros_->getCostmap();
		inscribed_radius_ = costmap_ros_->getLayeredCostmap()->getInscribedRadius();
		std::cout << "INITIALIZE PUBLISHER" << std::endl;
		eval_time_publisher_ = parent->create_publisher<std_msgs::msg::Float64>("eval_time", 10);
		eval_time_publisher_->on_activate();

		getParams();
		resetBatches();
		RCLCPP_INFO(logger_, "Configured");
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getParams()
		-> void
	{

		auto getParam = [&](const std::string &param_name, auto default_value)
		{
			std::string name = node_name_ + '.' + param_name;
			return utils::getParam(name, default_value, parent_);
		};

		model_dt_ = getParam("model_dt", 0.1);
		time_steps_ = getParam("time_steps", 15);
		batch_size_ = getParam("batch_size", 200);
		iteration_count_ = getParam("iteration_count", 2);

		v_std_ = getParam("v_std", 0.1);
		w_std_ = getParam("w_std", 0.3);
		v_limit_ = getParam("v_limit", 0.5);
		w_limit_ = getParam("w_limit", 1.3);
		max_v_acc_ = getParam("max_v_acc", 1.0);
		max_w_acc_ = getParam("max_w_acc", 2.0);
		
		temperature_ = getParam("temperature", 0.25);

		backward_motion_cost_weight_ = getParam("backward_motion_cost_weight", 1.0);

		reference_cost_power_ = getParam("reference_cost_power", 1.0);
		reference_cost_weight_ = getParam("reference_cost_weight", 5.0);

		goal_cost_power_ = getParam("goal_cost_power", 1.0);
		goal_cost_weight_ = getParam("goal_cost_weight", 20.0);

		goal_angle_cost_power_ = getParam("goal_angle_cost_power", 1.0);
		goal_angle_cost_weight_ = getParam("goal_angle_cost_weight", 10.0);

		obstacle_cost_power_ = getParam("obstacle_cost_power", 2.0);
		obstacle_cost_weight_ = getParam("obstacle_cost_weight", 10.0);

		inflation_cost_scaling_factor_ = getParam("inflation_cost_scaling_factor", 3.0);
		inflation_radius_ = getParam("inflation_radius", 0.75);
		threshold_to_consider_goal_angle_ = getParam("threshold_to_consider_goal_angle", 0.30);

		approx_reference_cost_ = getParam("approx_reference_cost", false);

		slope_cost_power_ = getParam("slope_cost_power", 1.0);
		slope_cost_weight_ = getParam("slope_cost_weight", 1.0);
		slope_crit_ = getParam("slope_crit", 0.5);
		roughness_cost_power_ = getParam("roughness_cost_power", 1.0);
		roughness_cost_weight_ = getParam("roughness_cost_weight", 1.0);
		roughness_crit_ = getParam("roughness_crit", 0.5);

		slope_traversability_delta_ = getParam("slope_traversability_delta", 0.1);
		slope_traversability_alpha_ = getParam("slope_traversability_alpha", 0.5);
		slope_traversability_lambda_ = getParam("slope_traversability_lambda", 1.0);

		slope_traversability_cost_power_ = getParam("slope_traversability_cost_power", 1.0);
		slope_traversability_cost_weight_ = getParam("slope_traversability_cost_weight", 10.0);

		obstacle_avoidance_method_ = getParam("obstacle_avoidance_method", COSTMAP_METHOD);
		footprint_radius_ = getParam("footprint_radius", inscribed_radius_);

		predefined_trajectories_on_cold_start_ = getParam("predefined_trajectories_on_cold_start", true);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::resetBatches()
		-> void
	{
		batches_ = xt::zeros<T>({batch_size_, time_steps_, last_dim_size_});
		control_sequence_ = xt::zeros<T>({time_steps_, control_dim_size_});
		xt::view(batches_, xt::all(), xt::all(), 4) = model_dt_;
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::generateNoisedTrajectories(
		const geometry_msgs::msg::PoseStamped &pose,
		const geometry_msgs::msg::Twist &twist)
		-> xt::xtensor<T, 3>
	{

		getBatchesControls() = generateNoisedControlBatches();

		applyControlConstraints();

		evalBatchesVelocities(twist);

		return integrateBatchesVelocities(pose);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::generateNoisedControlBatches() const
		-> xt::xtensor<T, 3>
	{
		xt::xtensor<T, 3> v_noises =
			xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, v_std_);
		xt::xtensor<T, 3> w_noises =
			xt::random::randn<T>({batch_size_, time_steps_, 1}, 0.0, w_std_);
		
		if (predefined_trajectories_on_cold_start_ && (xt::amax(xt::abs(control_sequence_))[0] < 0.01))
		{
			RCLCPP_INFO(logger_, "Zero control sequence. Predefined trajectories was added");
			addStrajghtTrajectories(v_noises, w_noises);
		}
		return control_sequence_ + xt::concatenate(xt::xtuple(v_noises, w_noises), 2);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::addStrajghtTrajectories(xt::xtensor<T, 3> &v_noises,
													  xt::xtensor<T, 3> &w_noises) const
		-> void
	{

		// TODO: Compute trajectories for variable number of steps;


		for (int i = 0; i < 10 && i < time_steps_; i++)
		{
			xt::view(v_noises, i, xt::all(), 0) = v_limit_ * 0.1 * (i + 1);
			xt::view(w_noises, i, xt::all(), 0) = 0;
		}
		for (int i = 10; i < 20 && i < time_steps_; i++)
		{
			xt::view(v_noises, i, xt::all(), 0) = -v_limit_ * 0.1 * (i - 9);
			xt::view(w_noises, i, xt::all(), 0) = 0;
		}
		for (int i = 20; i < 30 && i < time_steps_; i++)
		{
			xt::view(v_noises, i, xt::all(), 0) = 0;
			xt::view(w_noises, i, xt::all(), 0) = w_limit_ * 0.1 * (i - 19);
		}
		for (int i = 30; i < 40 && i < time_steps_; i++)
		{
			xt::view(v_noises, i, xt::all(), 0) = 0;
			xt::view(w_noises, i, xt::all(), 0) = -w_limit_ * 0.1 * (i - 29);
		}
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::applyControlConstraints()
		-> void
	{
		auto v = getBatchesControlLinearVelocities();
		auto w = getBatchesControlAngularVelocities();

		v = xt::clip(v, -v_limit_, v_limit_);
		w = xt::clip(w, -w_limit_, w_limit_);
		for (int i = 1; i < time_steps_; i++)
		{
			auto v_i = xt::view(v, xt::all(), i);
			auto v_prev = xt::view(v, xt::all(), i - 1);
			double dv = max_v_acc_ * model_dt_;
			xt::view(v, xt::all(), i) = xt::clip(v_i, v_prev - dv, v_prev + dv);
			auto w_i = xt::view(w, xt::all(), i);
			auto w_prev = xt::view(w, xt::all(), i - 1);
			double dw = max_w_acc_ * model_dt_;
			xt::view(w, xt::all(), i) = xt::clip(w_i, w_prev - dw, w_prev + dw);
		}
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::evalBatchesVelocities(const geometry_msgs::msg::Twist &twist)
		-> void
	{
		setBatchesInitialVelocities(twist);
		propagateBatchesVelocitiesFromInitials();
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::setBatchesInitialVelocities(const geometry_msgs::msg::Twist &twist)
		-> void
	{
		xt::view(batches_, xt::all(), 0, 0) = twist.linear.x;
		xt::view(batches_, xt::all(), 0, 1) = twist.angular.z;
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::propagateBatchesVelocitiesFromInitials()
		-> void
	{
		using namespace xt::placeholders;

		for (int t = 0; t < time_steps_ - 1; t++)
		{
			auto curr_batch = xt::view(batches_, xt::all(), t); // -> batch x 5
			auto next_batch_velocities =
				xt::view(batches_, xt::all(), t + 1, xt::range(_, 2)); // batch x 2
			next_batch_velocities = model_(curr_batch);
		}
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::evalTrajectoryFromControlSequence(
		const geometry_msgs::msg::PoseStamped &robot_pose,
		const geometry_msgs::msg::Twist &robot_speed) const
		-> xt::xtensor<T, 2>
	{
		auto &&batch = xt::xtensor<T, 2>::from_shape(
			{static_cast<size_t>(time_steps_),
			 static_cast<size_t>(last_dim_size_)});

		propagateSequenceVelocities(control_sequence_, robot_speed, batch);

		return integrateSequence(
			xt::view(batch, xt::all(), xt::range(0, 2)),
			robot_pose);
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::propagateSequenceVelocities(
		const auto &velocities_sequence,
		const geometry_msgs::msg::Twist &initial_speed,
		auto &batch) const
	{
		xt::view(batch, 0, 0) = initial_speed.linear.x;
		xt::view(batch, 0, 1) = initial_speed.angular.z;
		xt::view(batch, xt::all(), xt::range(2, 4)) = velocities_sequence;
		xt::view(batch, xt::all(), 4) = model_dt_;
		xt::view(batch, xt::all(), xt::range(0, 2)) = model_(batch);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::integrateSequence(
		const auto &velocities_sequence,
		const geometry_msgs::msg::PoseStamped &pose) const
		-> xt::xtensor<T, 2>
	{
		using namespace xt::placeholders;

		auto v = xt::view(velocities_sequence, xt::all(), 0);
		auto w = xt::view(velocities_sequence, xt::all(), 1);

		auto yaw = xt::cumsum(w * model_dt_, 0);

		xt::view(yaw, xt::range(1, _)) = xt::view(yaw, xt::range(_, -1));
		xt::view(yaw, xt::all()) += tf2::getYaw(pose.pose.orientation);

		auto v_x = v * xt::cos(yaw);
		auto v_y = v * xt::sin(yaw);

		auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 0);
		auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 0);

		return xt::concatenate(
			xt::xtuple(
				xt::view(x, xt::all(), xt::newaxis()),
				xt::view(y, xt::all(), xt::newaxis()),
				xt::view(yaw, xt::all(), xt::newaxis())),
			1);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::integrateBatchesVelocities(
		const geometry_msgs::msg::PoseStamped &pose) const
		-> xt::xtensor<T, 3>
	{
		using namespace xt::placeholders;

		auto v = getBatchesLinearVelocities();
		auto w = getBatchesAngularVelocities();
		double initial_yaw = tf2::getYaw(pose.pose.orientation);
		xt::xtensor<T, 2> yaw = initial_yaw + xt::cumsum(w * model_dt_, 1);

		auto yaw_offseted = yaw;
		xt::view(yaw_offseted, xt::all(), xt::range(1, _)) =
			xt::eval(xt::view(yaw, xt::all(), xt::range(_, -1)));
		xt::view(yaw_offseted, xt::all(), 0) = initial_yaw;

		auto v_x = v * xt::cos(yaw);
		auto v_y = v * xt::sin(yaw);

		auto x = pose.pose.position.x + xt::cumsum(v_x * model_dt_, 1);
		auto y = pose.pose.position.y + xt::cumsum(v_y * model_dt_, 1);

		return xt::concatenate(
			xt::xtuple(
				xt::view(x, xt::all(), xt::all(), xt::newaxis()),
				xt::view(y, xt::all(), xt::all(), xt::newaxis()),
				xt::view(yaw, xt::all(), xt::all(), xt::newaxis())),
			2);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::evalBatchesCosts(
		const xt::xtensor<T, 3> &batches_of_trajectories,
		const nav_msgs::msg::Path &global_plan,
		const geometry_msgs::msg::PoseStamped &robot_pose) const
		-> xt::xtensor<T, 1>
	{
		using namespace xt::placeholders;

		xt::xtensor<T, 1> costs = xt::zeros<T>({batch_size_});

		if (global_plan.poses.empty())
		{
			return costs;
		}

		auto &&path_tensor = geometry::toTensor<T>(global_plan);

		evalBackwardMotionCost(batches_of_trajectories, costs);
		// std::cout << "Backward motion cost: " << costs(0) << std::endl;

		approx_reference_cost_ ? evalApproxReferenceCost(batches_of_trajectories, path_tensor, costs) : evalReferenceCost(batches_of_trajectories, path_tensor, costs);
		// std::cout << "Plus reference cost: " << costs(0) << std::endl;

		evalGoalCost(batches_of_trajectories, path_tensor, costs);
		// std::cout << "Plus goal cost: " << costs(0) << std::endl;

		evalGoalAngleCost(batches_of_trajectories, path_tensor, robot_pose, costs);
		// std::cout << "Plus goal angle cost: " << costs(0) << std::endl;
		// auto t4 = std::chrono::high_resolution_clock::now();

		if (obstacle_avoidance_method_ == COSTMAP_METHOD)
		{
			evalObstacleCost(batches_of_trajectories, costs);
		}
		else if (obstacle_avoidance_method_ == SLOPE_TRAVERSABILITY_METHOD)
		{
			evalSlopeTraversabilityCost(batches_of_trajectories, robot_pose, costs);
		}
		else if (obstacle_avoidance_method_ == SLOPE_ROUGHNESS_METHOD)
		{
			evalSlopeRoughnessCost(batches_of_trajectories, costs);
		}
		else
		{
			evalObstacleCost(batches_of_trajectories, costs);
		}
		// std::cout << "Plus obstacle cost: " << costs(0) << std::endl;
		// auto t5 = std::chrono::high_resolution_clock::now();
		// auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(t5 - t4);
		// std::cout << "Time for eval slope roughness cost: " << dt.count() << std::endl;
		return costs;
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalBackwardMotionCost(
		const auto &batches_of_trajectories_points,
		auto &costs) const
	{
		using namespace xt::placeholders;

		xt::xtensor<T, 2> x_diff = xt::view(batches_of_trajectories_points, xt::all(), xt::range(1, _), 0) -
								xt::view(batches_of_trajectories_points, xt::all(), xt::range(_, -1), 0);
		xt::xtensor<T, 2> y_diff = xt::view(batches_of_trajectories_points, xt::all(), xt::range(1, _), 1) -
								xt::view(batches_of_trajectories_points, xt::all(), xt::range(_, -1), 1);

		xt::xtensor<T, 2> yaws = xt::view(batches_of_trajectories_points, xt::all(), xt::range(_, -1), 2);
		xt::xtensor<T, 2> thetas = xt::atan2(y_diff, x_diff) - yaws - M_PI;
		xt::xtensor<T, 2> v_negative = xt::cos(thetas) * xt::sqrt(x_diff * x_diff + y_diff * y_diff);
		v_negative = xt::maximum(v_negative, 0);
		costs += backward_motion_cost_weight_ * xt::sum(v_negative, 1);
		
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalSlopeTraversabilityCost(
		const auto &batches_of_trajectories_points,
		const geometry_msgs::msg::PoseStamped &robot_pose,
		auto &costs) const
	{
		size_t rows, cols;
		rows = grid_map_->getSize()[0];
		cols = grid_map_->getSize()[1];
		xt::xtensor<T, 2> slope_traversability_grid_ = -2 * xt::ones<T>({rows, cols});

		constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

		auto robotOnUntraversable = [this](const T &dist)
		{
			return dist < this->footprint_radius_;
		};

		for (size_t i = 0; i < static_cast<size_t>(batch_size_); i++)
		{

			double min_dist = std::numeric_limits<T>::max();
			bool is_closest_point_inflated = false;

			for (size_t j = 0; j < static_cast<size_t>(time_steps_); j++)
			{

				T traj_pos_x = batches_of_trajectories_points(i, j, 0);
				T traj_pos_y = batches_of_trajectories_points(i, j, 1);
				T dist_to_untraversable;

				if (minDistToUntraversableFromPose(traj_pos_x, traj_pos_y, slope_traversability_grid_, dist_to_untraversable))
				{
					if (robotOnUntraversable(dist_to_untraversable) && (j > 1))
					{
						is_closest_point_inflated = false;
						costs[i] = collision_cost_value;
						break;
					}
					if (dist_to_untraversable < min_dist)
					{
						is_closest_point_inflated = true;
						min_dist = dist_to_untraversable;
					}
				}
			}

			if (is_closest_point_inflated)
			{
				costs[i] += pow((1.01 * inflation_radius_ - min_dist) * slope_traversability_cost_weight_, slope_traversability_cost_power_);
			}
		}
	}

	template <typename T, typename Model>
	bool Optimizer<T, Model>::minDistToUntraversableFromPose(
		const T &x,
		const T &y,
		auto &slope_traversability_dist_grid_,
		T &dist) const
	{

		grid_map::Position center(x, y);
		grid_map::Index center_ij;
		if (not grid_map_->getIndex(center, center_ij))
		{
			dist = 0.0;
			return true;
		}

		if (slope_traversability_dist_grid_(center_ij[0], center_ij[1]) > -2.0)
		{
			if (slope_traversability_dist_grid_(center_ij[0], center_ij[1]) < 0.0)
			{
				dist = std::numeric_limits<T>::max();
				return false;
			}
			dist = slope_traversability_dist_grid_(center_ij[0], center_ij[1]);
			return true;
		}

		auto isUntraversable = [this](const grid_map::Index &cell)
		{
			std::string layer_name = "elevation";
			grid_map::Position cell_pos, p1, p2;
			grid_map::Index i1, i2;
			auto slope_xy = std::array<T, 2>();
			grid_map_->getPosition(cell, cell_pos);

			for (size_t i = 0; i < 2; i++)
			{
				p1 = cell_pos;
				p2 = cell_pos;
				p1[i] -= this->slope_traversability_delta_;
				p2[i] += this->slope_traversability_delta_;

				if (grid_map_->getIndex(p1, i1) and grid_map_->getIndex(p2, i2))
				{
					slope_xy[i] = std::abs((grid_map_->at(layer_name, i1) - grid_map_->at(layer_name, i2))) / (2 * slope_traversability_delta_);
				}
				else
				{
					return true;
				}
			}

			double slope = slope_traversability_alpha_ * slope_xy[0] +
						   (1 - slope_traversability_alpha_) * slope_xy[1];

			double traversable = std::exp(-slope_traversability_lambda_ * slope);

			if (std::isnan(traversable))
			{
				return true;
			}

			return not(traversable > 0.5);
		};

		bool obstacle_found = false;
		double min_dist = std::numeric_limits<T>::max();

		double radius = inflation_radius_;

		for (grid_map::CircleIterator iterator(*grid_map_, center, radius); !iterator.isPastEnd(); ++iterator)
		{
			if (isUntraversable(*iterator))
			{
				obstacle_found = true;
				grid_map::Position point;
				grid_map_->getPosition(*iterator, point);
				T dist_to_obst = (point - center).norm();
				if (dist_to_obst < min_dist)
				{
					min_dist = dist_to_obst;
				}
			}
		}

		dist = min_dist;

		if (obstacle_found)
		{
			slope_traversability_dist_grid_(center_ij[0], center_ij[1]) = dist;
		}
		else
		{
			slope_traversability_dist_grid_(center_ij[0], center_ij[1]) = -1.0;
		}

		return obstacle_found;
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalSlopeRoughnessCost(
		const auto &batches_of_trajectories_points,
		auto &costs) const
	{
		constexpr T unknown_cost_value = std::numeric_limits<T>::max() / 2;
		constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

		xt::xtensor<T, 3> slope_roughness = xt::zeros<T>({batch_size_, time_steps_, 2});

		xt::xtensor<T, 1> slope_values = xt::zeros<T>({time_steps_});
		xt::xtensor<T, 1> roughness_values = xt::zeros<T>({time_steps_});

		T slope, roughness;

		xt::xtensor<T, 1> slp_cost = xt::zeros<T>({slope_roughness.shape()[0]});
		xt::xtensor<T, 1> rgh_cost = xt::zeros<T>({slope_roughness.shape()[0]});

		grid_map::Size grid_map_size = grid_map_->getSize();
		xt::xtensor<T, 2> slope_at_grid = xt::ones<T>({grid_map_size(0), grid_map_size(1)}) * (-1);
		xt::xtensor<T, 2> roughness_at_grid = xt::ones<T>({grid_map_size(0), grid_map_size(1)}) * (-1);

		for (size_t batch = 0; batch < static_cast<size_t>(batch_size_); batch++)
		{
			bool unknown = false;
			for (size_t step = 0; step < static_cast<size_t>(time_steps_); step++)
			{
				double x = batches_of_trajectories_points(batch, step, 0);
				double y = batches_of_trajectories_points(batch, step, 1);
				grid_map::Index index;
				grid_map::Position point(x, y);
				if (!grid_map_->getIndex(point, index))
				{
					// std::cout << "UNABLE TO GET INDEX FROM POSITION " << x << ' ' << y << std::endl;
					costs(batch) = unknown_cost_value;
					unknown = true;
					break;
				}
				else
				{
					if ((slope_at_grid(index(0), index(1)) < 0) && (roughness_at_grid(index(0), index(1)) < 0) && slopeRoughnessAtPose(x, y, slope, roughness))
					{
						slope_at_grid(index(0), index(1)) = slope;
						roughness_at_grid(index(0), index(1)) = roughness;
					}

					slope = slope_at_grid(index(0), index(1));
					roughness = roughness_at_grid(index(0), index(1));
					if ((slope >= 0) && (roughness >= 0))
					{
						slope_roughness(batch, step, 0) = slope;
						slope_roughness(batch, step, 1) = roughness;
						slope_values(step) = slope;
						roughness_values(step) = roughness;
					}
					else
					{
						if (step > 0)
						{
							costs(batch) = unknown_cost_value;
							unknown = true;
							break;
						}
					}
				}
			}

			if (not unknown)
			{
				slp_cost(batch) = xt::amax(slope_values)[0];
				rgh_cost(batch) = xt::amax(roughness_values)[0];
				double slp_cost_of_tail = xt::amax(xt::view(slope_values, xt::range(2, xt::placeholders::_)))[0];
				double rgh_cost_of_tail = xt::amax(xt::view(roughness_values, xt::range(2, xt::placeholders::_)))[0];

				if(slp_cost_of_tail > slope_crit_ or rgh_cost_of_tail > roughness_crit_)
				{
					costs(batch) = collision_cost_value;
				}
				else
				{
					costs(batch) += std::pow(slp_cost(batch), slope_cost_power_) * slope_cost_weight_;
					costs(batch) += std::pow(rgh_cost(batch), roughness_cost_power_) * roughness_cost_weight_;
				}
			}
		}
	}

	template <typename T, typename Model>
	bool Optimizer<T, Model>::slopeRoughnessAtPose(const T &x, const T &y, T &slp, T &rgh) const
	{
		xt::xtensor<T, 2> footprint = footprintPointsAtPose(x, y);

		if (footprint.shape()[0] < 4)
		{
			return false;
		}

		T a, b, c, resid;

		fitPlane(footprint, a, b, c, resid);

		xt::xtensor<T, 1> normal = {a, b, -1};
		normal = normal / xt::linalg::norm(normal, 2);

		rgh = sqrt(resid / static_cast<T>(footprint.shape()[0]));
		xt::xtensor<T, 1> uni_z = {0, 0, 1};
		slp = acos(abs(xt::linalg::dot(normal, uni_z)(0)));

		if (std::isnan(rgh) or std::isnan(slp))
		{
			return false;
		}

		return true;
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::fitPlane(
		const xt::xtensor<T, 2> &points,
		T &a,
		T &b,
		T &c,
		T &sum_resid) const
	{
		xt::xtensor<T, 2> g = xt::ones_like(points);
		xt::view(g, xt::all(), xt::range(0, 2)) = xt::view(points, xt::all(), xt::range(0, 2));

		std::vector<int> sh = {points.shape()[0], 1};
		xt::xtensor<T, 2> z = xt::zeros<T>(sh);
		xt::view(z, xt::all(), 0) = xt::view(points, xt::all(), 2);

		auto lstsq_res = xt::linalg::lstsq(g, z);

		a = std::get<0>(lstsq_res)(0, 0);
		b = std::get<0>(lstsq_res)(1, 0);
		c = std::get<0>(lstsq_res)(2, 0);
		sum_resid = std::get<1>(lstsq_res)(0);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::footprintPointsAtPose(const T &x, const T &y) const
		-> xt::xtensor<T, 2>
	{

		/** TODO
		- GridMap layer name selection;
		- Improve border cases processing;
		- Not circle shape selection;
		**/

		std::string layer_name = "elevation";
		grid_map::Position center(x, y);
		double radius = footprint_radius_;
		std::vector<T> points = std::vector<T>();

		bool footprint_inside_map = grid_map_->isInside(grid_map::Position(x + radius, y)) &&
									grid_map_->isInside(grid_map::Position(x - radius, y)) &&
									grid_map_->isInside(grid_map::Position(x, y + radius)) &&
									grid_map_->isInside(grid_map::Position(x, y - radius));

		if (not footprint_inside_map)
		{
			return xt::xtensor<T, 2>();
		}

		for (grid_map::CircleIterator iterator(*grid_map_, center, radius); !iterator.isPastEnd(); ++iterator)
		{
			grid_map::Position point;
			grid_map_->getPosition(*iterator, point);
			points.push_back(point[0]);
			points.push_back(point[1]);
			points.push_back(grid_map_->at(layer_name, *iterator));
		}

		std::vector<std::size_t> shape = {points.size() / 3, 3};
		xt::xtensor<T, 2> footprint = xt::adapt(std::move(points), shape);

		return footprint;
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalGoalCost(
		const auto &batches_of_trajectories,
		const auto &global_plan,
		auto &costs) const
	{
		const auto goal_points = xt::view(global_plan, -1, xt::range(0, 2));

		auto last_timestep_points = xt::view(
			batches_of_trajectories,
			xt::all(), -1, xt::range(0, 2));

		auto dim = last_timestep_points.dimension() - 1;

		auto &&batches_last_to_goal_dists = xt::norm_l2(
			std::move(
				last_timestep_points) -
				goal_points,
			{dim});

		costs += xt::pow(std::move(batches_last_to_goal_dists) * goal_cost_weight_, goal_cost_power_);
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalApproxReferenceCost(
		const auto &batches_of_trajectories,
		const auto &global_plan,
		auto &costs) const
	{
		auto path_points = xt::view(global_plan, xt::all(), xt::range(0, 2));
		auto batch_of_lines =
			xt::view(batches_of_trajectories, xt::all(), xt::all(), xt::newaxis(), xt::range(0, 2));
		auto dists = xt::norm_l2(path_points - batch_of_lines, {batch_of_lines.dimension() - 1});
		auto &&cost = xt::mean(xt::amin(std::move(dists), 1), 1);
		costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalReferenceCost(
		const auto &batches_of_trajectories,
		const auto &global_plan,
		auto &costs) const
	{
		xt::xtensor<T, 3> path_to_batches_dists =
			geometry::distPointsToLineSegments2D(global_plan, batches_of_trajectories);
		xt::xtensor<T, 1> cost = xt::mean(
			xt::amin(
				std::move(path_to_batches_dists),
				1, xt::evaluation_strategy::immediate),
			1, xt::evaluation_strategy::immediate);
		costs += xt::pow(std::move(cost) * reference_cost_weight_, reference_cost_power_);
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalObstacleCost(
		const auto &batches_of_trajectories_points,
		auto &costs) const
	{
		constexpr T collision_cost_value = std::numeric_limits<T>::max() / 2;

		auto minDistToObstacle = [this](const auto cost)
		{
			return (-1.0 / inflation_cost_scaling_factor_) *
					   std::log(cost / (nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1)) +
				   inscribed_radius_;
		};

		for (size_t i = 0; i < static_cast<size_t>(batch_size_); ++i)
		{
			double min_dist = std::numeric_limits<T>::max();
			bool is_closest_point_inflated = false;

			for (size_t j = 0; j < static_cast<size_t>(time_steps_); ++j)
			{
				double cost = costAtPose(
					batches_of_trajectories_points(i, j, 0),
					batches_of_trajectories_points(i, j, 1));

				if (inCollision(cost))
				{
					costs[i] = collision_cost_value;
					is_closest_point_inflated = false;
					break;
				}
				else
				{
					if (cost != nav2_costmap_2d::FREE_SPACE)
					{
						double dist = minDistToObstacle(cost);
						if (dist < min_dist)
						{
							is_closest_point_inflated = true;
							min_dist = dist;
						}
					}
				}
			}

			if (is_closest_point_inflated)
			{
				costs[i] += pow(
					(1.01 * inflation_radius_ - min_dist) * obstacle_cost_weight_,
					obstacle_cost_power_);
			}
		}
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::evalGoalAngleCost(
		const auto &batch_of_trajectories,
		const auto &global_plan,
		const geometry_msgs::msg::PoseStamped &robot_pose,
		auto &costs) const
	{
		xt::xtensor<T, 1> tensor_pose = {static_cast<T>(robot_pose.pose.position.x),
										 static_cast<T>(robot_pose.pose.position.y)};

		auto path_points = xt::view(global_plan, -1, xt::range(0, 2));

		T points_to_goal_dists = xt::norm_l2(tensor_pose - path_points, {0})();

		if (points_to_goal_dists < threshold_to_consider_goal_angle_)
		{
			auto yaws = xt::view(batch_of_trajectories, xt::all(), xt::all(), 2);
			auto goal_yaw = xt::view(global_plan, -1, 2);

			costs += xt::pow(
				xt::mean(xt::abs(yaws - goal_yaw), {1}) * goal_angle_cost_weight_,
				goal_angle_cost_power_);
		}
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::updateControlSequence(const xt::xtensor<T, 1> &costs)
		-> void
	{
		auto &&costs_normalized =
			costs - xt::amin(costs, xt::evaluation_strategy::immediate);

		auto exponents = xt::eval(xt::exp(-1 / temperature_ * costs_normalized));

		auto softmaxes =
			exponents / xt::sum(exponents, xt::evaluation_strategy::immediate);

		auto softmaxes_expanded =
			xt::view(softmaxes, xt::all(), xt::newaxis(), xt::newaxis());

		control_sequence_ = xt::sum(getBatchesControls() * softmaxes_expanded, 0);
		constexpr T unknown_cost_value = std::numeric_limits<T>::max() / 2;
		T min_cost = xt::amin(costs)[0];
		if (min_cost >= unknown_cost_value)
		{
			control_sequence_ *= 0.;
			//std::cout << "LOCAL PATH NOT FOUND!!!" << std::endl;
			throw nav2_core::PlannerException("No legal trajectories found");
		}
	}

	template <typename T, typename Model>
	bool Optimizer<T, Model>::inCollision(unsigned char cost) const
	{
		if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
		{
			return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
				   cost != nav2_costmap_2d::NO_INFORMATION;
		}
		else
		{
			return cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
		}
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::costAtPose(const double &x, const double &y) const
		-> double
	{

		unsigned int mx, my;
		if (not costmap_->worldToMap(x, y, mx, my))
		{
			if (costmap_ros_->getLayeredCostmap()->isTrackingUnknown())
			{
				return static_cast<T>(nav2_costmap_2d::LETHAL_OBSTACLE); // May be should be replaced with unknown_cost_value
			}
			else
			{
				return static_cast<T>(nav2_costmap_2d::FREE_SPACE);
			}
		}
		return static_cast<T>(costmap_->getCost(mx, my));
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getControlFromSequence(const auto &stamp, const std::string &frame)
		-> geometry_msgs::msg::TwistStamped
	{
		return geometry::toTwistStamped(xt::view(control_sequence_, 0), stamp, frame);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesLinearVelocities() const
	{
		return xt::view(batches_, xt::all(), xt::all(), 0);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesAngularVelocities() const
	{
		return xt::view(batches_, xt::all(), xt::all(), 1);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControlLinearVelocities() const
	{
		return xt::view(batches_, xt::all(), xt::all(), 2);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControlAngularVelocities() const
	{
		return xt::view(batches_, xt::all(), xt::all(), 3);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControls() const
	{
		return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesLinearVelocities()
	{
		return xt::view(batches_, xt::all(), xt::all(), 0);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesAngularVelocities()
	{
		return xt::view(batches_, xt::all(), xt::all(), 1);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControls()
	{
		return xt::view(batches_, xt::all(), xt::all(), xt::range(2, 4));
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControlLinearVelocities()
	{
		return xt::view(batches_, xt::all(), xt::all(), 2);
	}

	template <typename T, typename Model>
	auto Optimizer<T, Model>::getBatchesControlAngularVelocities()
	{
		return xt::view(batches_, xt::all(), xt::all(), 3);
	}

	template <typename T, typename Model>
	void Optimizer<T, Model>::shiftControlSequence()
	{
		using namespace  xt::placeholders;		
		xt::view(control_sequence_, xt::range(_, -1), xt::all()) = xt::view(control_sequence_, xt::range(1, _), xt::all());
		control_sequence_(time_steps_ - 1, 0) = 0;
		control_sequence_(time_steps_ - 1, 1) = 0;
	}

} // namespace mppi::optimization