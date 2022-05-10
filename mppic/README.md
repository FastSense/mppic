# MPPI Local Planner for 2.5D Environment

![demo](./images/demo.gif)


## Description





Implementation of Model Predictive Path Integral control algorithm [[1](#references)] (including navigation in 2.5D environment). This is a ROS2 Nav2 controller (local trajectory planner), the main aim of which is to track a global path and avoid collisions with static obstacles. 

### MPPI Algorithm

The MPPI algorithm is based on the MPC approach and adopt its scheme of operation. At each iteration, a control strategy is generated based on the predictive model of the object (robot) and knowledge about the current state. The control is limited to a small time horizon to increase the performance. Controller choses the first action from the generated control and sends it for execution. After that, the new state is evaluated and the described steps are repeated. The general scheme of MPC methods is shown in the figure below

![mpc](./images/mpc.png)

The main difference between the MPPI  and MPC is the control generation approach. In the MPPI algorithm a batch of control action sequances  (linear and angular velocities) by given time horizon `time_steps` and duration of single step `dt` is randomly generated. Various restrictions are imposed on the control, including restrictions on the maximum amplitude and on the difference between two adjacent steps. Next, the model is used to to predict real velocities (linear and angular) for each time step by given controls and initial velocities (current Twist),
this can be explained as follows:

<div align="center"><img style="background: white;" src="./images/svg/chQpxULhiy.svg"></div>

where:

  - <img style="transform: translateY(0.1em); background: white;" src="./images/svg/AbkUQideXd.svg"> - velocities (linear and angular) of all batches at time step <img style="transform: translateY(0.1em); background: white;" src="./images/svg/BmYS9Bxa0s.svg">, shape  [`batch_size`, 2]
  - <img style="transform: translateY(0.1em); background: white;" src="./images/svg/7qagi2OPwo.svg"> - batches consisting current velocities (linear, angular), controls (linear, angular) and `dt`, shape [`batch_size`, 5] at time step <img style="transform: translateY(0.1em); background: white;" src="./images/svg/g4eiq4pBAE.svg">

After that, the velocities are integrated to obtain a batches of trajectories. Trajectories are evaluated and costs of the trajectories are used as the weights for corresponding sets of controls in the softmax function, considering that the greater the cost of the trajectory, the less the weight of the corresponding sequence of controls. 

To evaluate the obtained trajectories, a function consisting of the sum of components was used, each of which was responsible for evaluating individual characteristics of the trajectory. The final function consisted of the following set of components:

- Distance between the last point of the trajectory and the target point;
- Distance between the points of the trajectory and the points of the reference global path;
- Angle between the necessary direction of the robot at the target point and the directions of the robot at the last points of the trajectory;
- Penalty for driving with negative linear velocity;
- Minimal distance between points of trajectory and obstacles on costmap2d (optional)
- Evaluation of the roughness of the environment in which the trajectory passes (optional).

Each component <!-- $i$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/oJl7QAGwvB.svg"> in the sum is raised to some power <!-- $\alpha_i$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/TkyJbrrLDn.svg"> and multiplied by some weight <!-- $w_i$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/jjLzGLHJep.svg">

<!-- $$
Cost(\pi) = \sum_{i}{w_i \times cost_i(\pi)^{\alpha_i}}
$$ --> 

<div align="center"><img style="background: white;" src="./images/svg/mEn9myaepL.svg"></div>

There are two options for roughness evaluation: `Slope-Traversability` and `Slope-Roughness`.

When using the `Slope-Traversability` method, the traversability of map cells is estimated based on the height map. For these purposes, the [[2](#references)] approach is used. This approach is based on the calculation of the probability <img style="transform: translateY(0.1em); background: white;" src="./images/svg/rpUDAGzZzg.svg"> of traversability  for each cell <img style="transform: translateY(0.1em); background: white;" src="./images/svg/V8ktAmygRV.svg"> . For these purposes, the height difference between the cells located at a distance of <!-- $\Delta$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/e11BIW8Bt9.svg"> from the cell <!-- $(i, j)$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/BuJQBWoHFN.svg"> along the vertical and horizontal axes is calculated.

<div align="center"><img style="background: white;" src="./images/svg/13rZhP1Xza.svg"></div>
<div align="center"><img style="background: white;" src="./images/svg/Dqh8OTmSzD.svg"></div>
<div align="center"><img style="background: white;" src="./images/svg/8kDP7HBNm5.svg"></div>


The adjustment of this method to certain robot parameters is performed by varying the parameters <!-- $\Delta$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/qOUBBmkarw.svg"> and <!-- $\lambda$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/r0QFyMxbOv.svg">. In this case, the cell <!-- $(i, j)$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/8z1k6sr1Gm.svg"> is considered traversable if <!-- $p(i, j) > 0.5$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/sY3TmByjFf.svg">.

After the required area of the height map has been binarized, the distance to the nearest untraversable cell is calculated for each trajectory from the batches. Moreover, if the trajectory runs through an untraversable area, then the cost of this trajectory is set equal to the some huge allowable value (`numeric_limits<float>::max()/2`).


When using the `Slope-Roughness` method (partly described at [[3](#references)]), cost consists of two components. The first component evaluates the maximum slope of the robot during the execution of the trajectory <!-- $\pi$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/ehew4N1WFn.svg">, the second component evaluates the maximum surface roughness during the execution of the trajectory <!-- $\pi$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/Hk5qy60Ad6.svg">. To calculate both components, for each discrete point <!-- $\pi_i$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/lQH6S8oUWT.svg"> of the trajectory, a robot footprint area  <!-- $\mathcal{B}_r(\pi_i)$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/EwHejXQrnj.svg"> is constructed, inside which the coordinates <!-- $(x, y, z)$ --> <img style="transform: translateY(0.1em); background: white;" src="./images/svg/UoWIuFI7Da.svg"> for each cell of the height map are known. Based on this information, an best fitting plane is constructed using the least squares method. After that, the slope of the obtained plane is calculated and the root of the mean square of the deviation of the cells of the elevation map from the plane (estimation of the surface roughness).

<!-- $$
a_i, b_i, c_i = LeastSquareSolution(\{ (x_j, y_j, z_j) \in \mathcal{B}_r(\pi_i) \})
$$ --> 

<div align="center"><img style="background: white;" src="./images/svg/MBctolt0XV.svg"></div>

<!-- $$
S_{slope}(\pi) = \max_{\pi_i \in \pi} \arccos(\frac{1}{\sqrt{a_i^2 + b_i^2 + 1}}(a_i, b_i, -1) \cdot (0, 0, 1))
$$ --> 

<div align="center"><img style="background: white;" src="./images/svg/UrPCIDJMU6.svg"></div>

<!-- $$
\sigma_i = \sqrt{\frac{1}{|\mathcal{B}_r(\pi_i)|} \sum\limits_{(x_j, y_j, z_j) \in \mathcal{B}_r(\pi_i)} \frac{(a_i x_j + b_i y_j + c_i - z_j)^2}{a_i^2 + b_i^2 + 1}}
$$ --> 

<div align="center"><img style="background: white;" src="./images/svg/5DPPDV3Jlf.svg"></div>

<!-- $$
S_{rough}(\pi) = \max_{\pi_i \in \pi}  \sigma_i
$$ --> 

<div align="center"><img style="background: white;" src="./images/svg/RSbhpyBo0F.svg"></div>


### Implemenation Details

This plugin implements the `nav2_core::Controller` interface [[4](#references)] allowing it to be used across the navigation stack as a local trajectory planner in the controller server's action server (controller_server).

The standard costmap2d (for navigation in 2D) [[5](#references)], or a separate topic publishing grid_map (for navigation in 2.5D) [[6](#references)] can be used as a source of data about the environment. For matrix and tensor operations, the package uses the `xtensor` library. 

Repository also includes environment for standalone experimental execution of MPPI, Jupyter Notebook for proccessing experiment results and files of our results.


## Dependencies

MPPIc package requires 
- `git` 
- `C++` compiler supporting `C++17` and `CMake 3.5` or above
- `ROS2 Foxy` (including `TF2`)
- `ROS2 Nav2`
- `grid_map` package [[6](#references)]
- `Python3`, `SciPy`, `Jupyter Notebook` for experiment results processing

## Input Data

As input, the module receive a standard set necessary for the operation of the `nav2_core::Controller` (`tf2_ros::Buffer`, `nav2_costmap_2d::Costmap2DROS`). It also optionaly can receive `grid_map` messages from specified topic. 

### Parameters List

| Parameter       | Type   | Default | Definition|
| --------------- | ------ | ------- | ---------------------------------------------------------------- |
| iteration_count                  | int    | 2       | Iteration count in MPPI algorithm |
| grid_map_topic                   | string | elevation_map | Name of the topic with grid_map data |
| lookahead_dist                   | double | 1.0     | Max lenght of the global plan, considering by local planner |
| batch_size                       | int    | 200     | Count of randomly sampled trajectories |
| time_steps                       | int    | 15      | Number of points propagated in time in each sampled trajectory |
| model_dt                         | double | 0.1     | Time interval between two points in sampled trajectories|
| v_std                            | double | 0.1     | Standart deviation for linear speed sampling |
| w_std                            | double | 0.3     | Standart deviation for angular speed sampling |
| v_limit                          | double | 0.5     | Linear speed control limit |
| w_limit                          | double | 1.3     | Angular speed control limit |
| max_v_acc                        | double | 1.0     | Maximum acceleration of the linear velocity component |
| max_w_acc                        | double | 2.0     | Maximum acceleration of the angular velocity component |
| temperature                      | double | 0.25    | Selectiveness of trajectories by their costs (The closer this value to 0, the more we take controls with less cost), 0 mean use control with best cost, huge value will lead to just taking mean of all trajectories withou cost consideration |
| goal_cost_weight                 | double | 20.0    | |
| goal_cost_power                  | double | 1.0     | |
| reference_cost_weight            | double | 5.0     | |
| reference_cost_power             | double | 1.0     | |
| obstacle_cost_weight             | double | 10.0    | * use it in case of costmap2d obstacle avoidance ("costmap" method) |
| obstacle_cost_power              | double | 2.0     | * use it in case of costmap2d obstacle avoidance ("costmap" method) |
| goal_angle_cost_weight           | double | 10.0    | |
| goal_angle_cost_power            | double | 1.0     | |
| backward_motion_cost_weight      | double | 50.0     | |
| slope_cost_power                 | double | 1.0     | |
| slope_cost_weight                | double | 10.0     | |
| slope_crit                       | double | 0.2     | The critical value of the slope component of "slope_roughness" method, upon reaching which the trajectory is considered impassable |
| roughness_cost_power             | double | 1.0     | |
| roughness_cost_weight            | double | 10.0     | |
| roughness_crit                   | double | 0.01     | The critical value of the roughness component of "slope_roughness" method, upon reaching which the trajectory is considered impassable |
| slope_traversability_delta       | double | 0.1     | Δ in "slope_traversability" method |
| slope_traversability_alpha       | double | 0.5     | Propotrion between horizontal and vertical components in "slope_traversability" method|
| slope_traversability_lambda      | double | 3.0     | λ in "slope_traversability" method |
| slope_traversability_cost_power  | double | 1.0     | |
| slope_traversability_cost_weight | double | 20.0     | |
| obstacle_avoidance_method        | string | "slope_roughness"    |  Valid values: "slope_roughness", "slope_traversability", "costmap" |
| footprint_radius                 | double | inscribed_radius of costmap2d | Radius of the footprint for uneven navigation methods. |
| predefined_trajectories_on_cold_start | bool | true | |
| inflation_cost_scaling_factor    | double | 3       | Must be set accurately according to inflation layer params |
| inflation_radius                 | double | 0.75    | Must be set accurately according to inflation layer params |
| threshold_to_consider_goal_angle | double | 0.3     | Minimal distance between robot and goal above which angle goal cost considered |
| approx_reference_cost            | bool   | False   | Use approximate point to segment distance calculation |
| visualize                        | bool   | True    | Use visualization |

## Output Data

In addition to providing a control action in `cmd_vel` format for the `controller_server`, the plugin also publishes data to the following topics:

| Topic                     | Type                             | Description                                                           |
|---------------------------|----------------------------------|-----------------------------------------------------------------------|
| `trajectories`            | `visualization_msgs/MarkerArray` | Randomly generated trajectories, including resulting control sequence |
| `transformed_global_plan` | `nav_msgs/Path`                  | Part of global plan considered by local planner                       |


## Install, Build and Run

### Install

Clone this repository to your ROS2 workspace folder.

### Build

Build with required flags

```bash
colcon build --packages-select mppic --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON [build flags]
```
Build flags:

- `-DBUILD_TESTING=ON` - build unit tests
- `-DBUILD_BENCHMARKS=ON` - turn on benchmarks inside unit tests
- `-DBUILD_EXPERIMENTS=ON` - build executable for experimental evaluation
- `-DTEST_DEBUG_INFO=ON` - show additional console output


For example, build with unit test and additional console output:
```bash
colcon build --packages-select mppic --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DBUILD_TESTING=ON  -DTEST_DEBUG_INFO=ON
```

### Run

Include `FollowPath` as `controller_plugins` on Nav2 config `.yml` file. Add the parameters listed in the table above as `FollowPath` parameters inside `controller_server` parameters. Full instruction about setting up Nav2 plugins is placed at [[7](#references)].

Example fully-described `yml` with all parameters:

```yml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 1.0
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True

    FollowPath:
      plugin: "mppi::Controller<float>"
      grid_map_topic: "elevation_map"
      time_steps: 30
      lookahead_dist: 10.0
      model_dt: 0.1
      batch_size: 300
      v_std: 0.1
      w_std: 0.5
      v_limit: 0.5
      w_limit: 1.3
      max_v_acc: 1.0
      max_w_acc: 2.0
      iteration_count: 3
      temperature: 0.25
      backward_motion_cost_weight: 50.0
      reference_cost_power: 1.0
      reference_cost_weight: 1.0
      goal_cost_power: 1.0
      goal_cost_weight: 20.0
      goal_angle_cost_power: 1.0
      goal_angle_cost_weight: 5.0
      obstacle_cost_power: 2.0
      obstacle_cost_weight: 10.0
      inflation_cost_scaling_factor: 3.0
      inflation_radius: 0.75
      threshold_to_consider_goal_angle: 0.15
      approx_reference_cost: false
      visualize: true
      predefined_trajectories_on_cold_start: true
      
      obstacle_avoidance_method: "slope_roughness"
      footprint_radius: 0.3
      
      slope_cost_power: 1.0
      slope_cost_weight: 10.0
      slope_crit: 0.2
      roughness_cost_power: 1.0
      roughness_cost_weight: 10.0
      roughness_crit: 0.01

      slope_traversability_delta: 0.1
      slope_traversability_alpha: 0.5
      slope_traversability_lambda: 3.0
      slope_traversability_cost_power: 1.0
      slope_traversability_cost_weight: 20.0
```


### Run Tests

```bash
colcon test --packages-select mppic --ctest-args -VV --event-handlers console_direct+
```
### Run Experiments

```bash
ros2 run mppic experiments {input files}
```

Input `xml` files :
1. Config for experiment and mppi. 
2. Map description
3. Tasks description

Examples of such configs are placed at `experiments/experiments_processing/data` folder.

Example of running script:

```bash
ros2 run mppic experiments config.xml map.xml tasks.xml > output.txt
```

## References

1. [G. Williams, P. Drews, B. Goldfain, J. M. Rehg and E. A. Theodorou, "Aggressive driving with model predictive path integral control," 2016 IEEE International Conference on Robotics and Automation (ICRA), 2016, pp. 1433-1440](https://ieeexplore.ieee.org/document/7487277)
2. [J. Shin, D. Kwak, and K. Kwak, “Model Predictive Path Planning for an Autonomous Ground Vehicle in Rough Terrain,” Int. J. Control. Autom. Syst., vol. 19, no. 6, pp. 2224–2237, 2021](https://link.springer.com/content/pdf/10.1007/s12555-020-0267-2.pdf)
3. [A. Chilian and H. Hirschmüller, “Stereo camera based navigation of mobile robots on rough terrain,” in 2009 IEEE/RSJ International Conference on Intelligent Robots and Systems, 2009, pp. 4571–4576](https://ieeexplore.ieee.org/abstract/document/5354535)
4. [Nav2 Navigation Plugins](https://navigation.ros.org/plugins/index.html)
5. [Nav2 Costmap2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
6. [ANYbotics grid_map package](https://github.com/ANYbotics/grid_map/tree/foxy-devel)
7. [Setting Up Navigation Plugins](https://navigation.ros.org/setup_guides/algorithm/select_algorithm.html)