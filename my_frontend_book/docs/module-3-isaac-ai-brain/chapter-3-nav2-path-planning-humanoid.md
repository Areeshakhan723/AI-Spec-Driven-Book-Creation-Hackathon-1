---
sidebar_position: 3
title: "Chapter 3: Nav2 Path Planning for Humanoid Robots"
---

# Chapter 3: Nav2 Path Planning for Humanoid Robots

This chapter focuses on adapting the Navigation2 (Nav2) stack for humanoid robots. Unlike wheeled robots, humanoid robots have unique constraints related to bipedal locomotion, balance, and step planning that require specialized path planning approaches.

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the differences between wheeled and humanoid robot navigation
- Configure Nav2 for humanoid-specific constraints
- Implement step-aware path planning for bipedal locomotion
- Integrate balance and stability considerations into navigation

## Introduction to Humanoid Navigation Challenges

Humanoid robots face unique navigation challenges compared to wheeled robots:

- **Bipedal Locomotion**: Requires precise footstep planning
- **Balance Constraints**: Must maintain center of mass within support polygon
- **Step Height/Limitations**: Cannot step over large obstacles
- **Dynamic Stability**: Requires continuous balance control during movement
- **Foot Clearance**: Must maintain proper foot clearance during walking

## Nav2 Architecture for Humanoid Robots

### Standard Nav2 vs Humanoid Nav2

Standard Nav2 assumes continuous motion with differential or Ackermann steering, but humanoid robots require:

1. **Discrete Step Planning**: Instead of continuous motion, humanoid robots move in discrete steps
2. **Balance-Aware Planning**: Path planning must consider balance constraints
3. **Footstep Planning**: Each step must be planned with balance in mind
4. **Stability Verification**: Continuous verification of stability during execution

### Humanoid-Specific Nav2 Components

```yaml
# Humanoid-specific Nav2 configuration
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    default_bt_xml_filename: "humanoid_navigate_w_replanning_and_recovery.xml"
    plugin_lib_names:
    - nav2_compute_path_to_pose_action
    - nav2_follow_path_action
    - nav2_back_up_action
    - nav2_spin_action
    - nav2_wait_action
    - nav2_clear_costmap_service
    - nav2_is_stuck_condition
    - nav2_goal_reached_condition
    - nav2_goal_updated_condition
    - nav2_initial_pose_received_condition
    - nav2_reinitialize_global_localization_service
    - nav2_rate_controller
    - nav2_distance_controller
    - nav2_speed_controller
    - nav2_truncate_path_action
    - nav2_goal_updater_node
    - nav2_recovery_node
    - nav2_pipeline_sequence
    - nav2_round_robin_node
    - nav2_transform_available_condition
    - nav2_time_expired_condition
    - nav2_distance_traveled_condition
    - nav2_single_trigger
    - nav2_is_battery_low_condition
    - humanoid_footstep_planner
    - humanoid_balance_checker

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 50
      model_dt: 0.05
      batch_size: 1000
      ax_max: 3.0
      ax_min: -3.0
      ay_max: 3.0
      ay_min: -3.0
      ath_max: 1.0
      ath_min: -1.0
      vx_std: 0.2
      vy_std: 0.2
      vth_std: 0.2
      vx_max: 0.5  # Humanoid walking speed
      vx_min: -0.2
      vy_max: 0.3
      vy_min: -0.3
      vth_max: 0.5
      vth_min: -0.5
      xy_goal_tolerance: 0.2
      yaw_goal_tolerance: 0.2
      state_reset_tolerance: 0.5
      control_duration: 0.05
      motion_model: "humanoid_model"
      reference_heading_from_robot: true
      transform_tolerance: 0.1
      heading_lookahead_distance: 0.3
      oscillation_distance: 0.02
      oscillation_angle: 0.2
      cost_scaling_dist: 1.0
      cost_scaling_gain: 1.0
      inflation_cost_scaling: 3.0
      replan_frequency: 0.1
      use_astar: false
      debug_multithreading: false
```

## Footstep Planning for Humanoid Robots

### Step Planning Fundamentals

Humanoid navigation requires planning each footstep explicitly:

```cpp
// Example: Humanoid footstep planning
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

struct Footstep {
    geometry_msgs::msg::PoseStamped pose;
    double support_time;  // Time for this step
    bool is_left_foot;    // Which foot for this step
};

class HumanoidFootstepPlanner {
public:
    HumanoidFootstepPlanner() {
        // Initialize with humanoid-specific parameters
        step_height_ = 0.1;      // Height of foot during step
        step_length_ = 0.3;      // Maximum step length
        step_width_ = 0.2;       // Distance between feet
        min_step_time_ = 0.5;    // Minimum time per step
    }

    std::vector<Footstep> planFootsteps(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {

        std::vector<Footstep> footsteps;

        // Calculate path between start and goal
        nav_msgs::msg::Path path = calculatePath(start, goal);

        // Convert path to footsteps
        footsteps = convertPathToFoosteps(path);

        // Verify balance constraints for each step
        footsteps = verifyBalanceConstraints(footsteps);

        return footsteps;
    }

private:
    std::vector<Footstep> convertPathToFoosteps(const nav_msgs::msg::Path& path) {
        std::vector<Footstep> footsteps;

        // For each segment of the path, plan appropriate footsteps
        for (size_t i = 0; i < path.poses.size(); i += 2) {
            Footstep step;
            step.pose = path.poses[i];
            step.support_time = calculateStepTime(path.poses[i], path.poses[i+1]);
            step.is_left_foot = (i / 2) % 2 == 0;  // Alternate feet

            footsteps.push_back(step);
        }

        return footsteps;
    }

    std::vector<Footstep> verifyBalanceConstraints(const std::vector<Footstep>& footsteps) {
        std::vector<Footstep> valid_footsteps;

        for (const auto& step : footsteps) {
            if (isBalanceMaintained(step, valid_footsteps)) {
                valid_footsteps.push_back(step);
            } else {
                // Adjust step to maintain balance
                Footstep adjusted_step = adjustForBalance(step, valid_footsteps);
                valid_footsteps.push_back(adjusted_step);
            }
        }

        return valid_footsteps;
    }

    bool isBalanceMaintained(const Footstep& step, const std::vector<Footstep>& previous_steps) {
        // Check if the step maintains the robot's center of mass within support polygon
        // Implementation would check ZMP (Zero Moment Point) or other balance metrics
        return true;  // Simplified for example
    }

    double step_height_;
    double step_length_;
    double step_width_;
    double min_step_time_;
};
```

### Balance-Aware Path Planning

Humanoid robots must maintain balance during navigation:

```python
# Balance-aware path planning for humanoid robots
import numpy as np
from scipy.spatial.transform import Rotation as R

class BalanceAwarePathPlanner:
    def __init__(self):
        self.com_height = 0.8  # Center of mass height in meters
        self.foot_separation = 0.3  # Distance between feet
        self.max_lean_angle = 15.0  # Maximum lean in degrees
        self.zmp_margin = 0.05  # Safety margin for ZMP

    def plan_balanced_path(self, start_pose, goal_pose, map_resolution=0.05):
        """
        Plan a path that maintains balance constraints for humanoid robot
        """
        # Convert poses to coordinates
        start_pos = np.array([start_pose.pose.position.x,
                             start_pose.pose.position.y])
        goal_pos = np.array([goal_pose.pose.position.x,
                            goal_pose.pose.position.y])

        # Plan initial path using standard algorithm
        path = self.plan_initial_path(start_pos, goal_pos)

        # Verify and adjust for balance constraints
        balanced_path = self.verify_balance_constraints(path)

        return balanced_path

    def verify_balance_constraints(self, path):
        """
        Verify that the path maintains balance for humanoid robot
        """
        balanced_path = []

        for i, pose in enumerate(path):
            if self.is_balanced(pose, path, i):
                balanced_path.append(pose)
            else:
                # Adjust pose to maintain balance
                adjusted_pose = self.adjust_for_balance(pose, path, i)
                balanced_path.append(adjusted_pose)

        return balanced_path

    def is_balanced(self, pose, path, current_index):
        """
        Check if a pose maintains balance for humanoid robot
        """
        # Calculate Zero Moment Point (ZMP) for this pose
        zmp = self.calculate_zmp(pose, path, current_index)

        # Check if ZMP is within support polygon
        support_polygon = self.calculate_support_polygon(pose, path, current_index)

        # Check if ZMP is within polygon with safety margin
        is_stable = self.is_point_in_polygon(zmp, support_polygon, self.zmp_margin)

        return is_stable

    def calculate_zmp(self, pose, path, current_index):
        """
        Calculate Zero Moment Point for the current pose
        """
        # Simplified ZMP calculation
        # In reality, this would consider dynamic forces
        zmp_x = pose.pose.position.x
        zmp_y = pose.pose.position.y

        return np.array([zmp_x, zmp_y])

    def calculate_support_polygon(self, pose, path, current_index):
        """
        Calculate support polygon based on foot positions
        """
        # For a humanoid, the support polygon is typically the convex hull
        # of the current stance foot (or both feet if standing)
        foot_positions = self.get_foot_positions(pose, path, current_index)

        return self.convex_hull(foot_positions)

    def get_foot_positions(self, pose, path, current_index):
        """
        Get current foot positions based on gait pattern
        """
        # Simplified - in reality, this would depend on the current gait phase
        left_foot = np.array([pose.pose.position.x + 0.1,
                             pose.pose.position.y + self.foot_separation/2])
        right_foot = np.array([pose.pose.position.x + 0.1,
                              pose.pose.position.y - self.foot_separation/2])

        return [left_foot, right_foot]

    def is_point_in_polygon(self, point, polygon, margin=0):
        """
        Check if a point is inside a polygon with optional margin
        """
        # Ray casting algorithm with margin
        x, y = point
        n = len(polygon)
        inside = False

        p1x, p1y = polygon[0]
        for i in range(n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y) and y <= max(p1y, p2y) and x <= max(p1x, p2x):
                if p1y != p2y:
                    xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                if p1x == p2x or x <= xinters:
                    inside = not inside
            p1x, p1y = p2x, p2y

        return inside

    def convex_hull(self, points):
        """
        Calculate convex hull of points (simplified)
        """
        # In practice, use scipy.spatial.ConvexHull
        return points

    def plan_initial_path(self, start, goal):
        """
        Plan initial path using A* or similar algorithm
        """
        # Simplified path planning
        path = [start]
        current = start.copy()

        while np.linalg.norm(current - goal) > 0.1:
            direction = (goal - current) / np.linalg.norm(goal - current)
            step = current + direction * 0.1  # 10cm steps
            path.append(step)
            current = step

        path.append(goal)
        return path
```

## Humanoid Navigation Behaviors

### Step-Aware Navigation

Humanoid robots need step-aware navigation behaviors:

```cpp
// Humanoid navigation behavior tree node
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

class HumanoidStepPlanner : public BT::ActionNodeBase
{
public:
    HumanoidStepPlanner(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config)
    {
        // Initialize with humanoid-specific parameters
        max_step_height_ = 0.15;
        max_step_width_ = 0.5;
        step_duration_ = 0.8;
    }

    BT::NodeStatus tick() override
    {
        // Get current goal
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput<geometry_msgs::msg::PoseStamped>("goal", goal)) {
            throw BT::RuntimeError("Missing required input [goal]");
        }

        // Plan footsteps to reach goal
        nav_msgs::msg::Path footstep_path = planFootstepsToGoal(goal);

        // Check if path is balanced
        if (isPathBalanced(footstep_path)) {
            // Set output path
            setOutput("footstep_path", footstep_path);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
            BT::OutputPort<nav_msgs::msg::Path>("footstep_path")
        };
    }

private:
    nav_msgs::msg::Path planFootstepsToGoal(const geometry_msgs::msg::PoseStamped& goal)
    {
        // Implementation of footstep planning algorithm
        nav_msgs::msg::Path path;

        // Calculate footsteps to reach goal
        // This would involve complex bipedal gait planning

        return path;
    }

    bool isPathBalanced(const nav_msgs::msg::Path& path)
    {
        // Check if the planned path maintains balance constraints
        for (const auto& pose : path.poses) {
            if (!isPoseBalanced(pose)) {
                return false;
            }
        }
        return true;
    }

    bool isPoseBalanced(const geometry_msgs::msg::PoseStamped& pose)
    {
        // Check balance at this pose
        // Implementation would check ZMP, support polygon, etc.
        return true;  // Simplified for example
    }

    double max_step_height_;
    double max_step_width_;
    double step_duration_;
};
```

### Obstacle Avoidance for Humanoid Robots

Humanoid robots have different obstacle avoidance requirements:

```python
# Humanoid-aware obstacle avoidance
import numpy as np

class HumanoidObstacleAvoider:
    def __init__(self):
        self.step_height_limit = 0.15  # Cannot step over obstacles higher than this
        self.step_width_limit = 0.5    # Cannot step across gaps wider than this
        self.foot_size = 0.25          # Approximate foot size
        self.com_height = 0.8          # Center of mass height

    def is_path_traversable(self, path, costmap, robot_radius=0.3):
        """
        Check if a path is traversable for a humanoid robot
        """
        for pose in path.poses:
            # Check if the position is safe for stepping
            if not self.is_step_safe(pose, costmap, robot_radius):
                return False

            # Check if the path maintains balance
            if not self.is_balance_maintained(pose, path):
                return False

        return True

    def is_step_safe(self, pose, costmap, robot_radius):
        """
        Check if a single step is safe for the humanoid robot
        """
        x = int(pose.pose.position.x / costmap.info.resolution)
        y = int(pose.pose.position.y / costmap.info.resolution)

        # Check if the position is within map bounds
        if x < 0 or x >= costmap.info.width or y < 0 or y >= costmap.info.height:
            return False

        # Check cost at this position (considering foot size)
        cost = self.get_foot_cost(x, y, costmap, robot_radius)

        # Path should have low cost for humanoid to step there
        return cost < 100  # Threshold for traversable

    def get_foot_cost(self, x, y, costmap, robot_radius):
        """
        Calculate cost considering the size of the humanoid's foot
        """
        # Check multiple points within the foot area
        foot_points = self.get_foot_area_points(x, y, costmap.info.resolution)
        max_cost = 0

        for fx, fy in foot_points:
            if 0 <= fx < costmap.info.width and 0 <= fy < costmap.info.height:
                cost_idx = fy * costmap.info.width + fx
                if cost_idx < len(costmap.data):
                    cost = costmap.data[cost_idx]
                    max_cost = max(max_cost, cost)

        return max_cost

    def get_foot_area_points(self, x, y, resolution):
        """
        Get points that represent the area of a humanoid foot
        """
        points = []
        foot_radius = self.foot_size / 2.0 / resolution  # Convert to grid units

        for dx in range(int(-foot_radius), int(foot_radius) + 1):
            for dy in range(int(-foot_radius), int(foot_radius) + 1):
                if dx*dx + dy*dy <= foot_radius*foot_radius:
                    points.append((x + dx, y + dy))

        return points

    def is_balance_maintained(self, pose, path):
        """
        Check if balance is maintained at this pose
        """
        # Implementation would check ZMP, support polygon, etc.
        return True  # Simplified for example

    def generate_alternative_path(self, original_path, costmap, robot_radius=0.3):
        """
        Generate alternative paths that consider humanoid constraints
        """
        alternative_paths = []

        # Try different approaches to navigate around obstacles
        for approach in ['step-around', 'step-over', 'step-across']:
            try:
                path = self.plan_approach(original_path, approach, costmap, robot_radius)
                if self.is_path_traversable(path, costmap, robot_radius):
                    alternative_paths.append(path)
            except:
                continue  # Skip if approach fails

        return alternative_paths

    def plan_approach(self, original_path, approach, costmap, robot_radius):
        """
        Plan a path using a specific approach (step-around, step-over, etc.)
        """
        # Implementation would vary based on approach
        return original_path  # Simplified for example
```

## Integration with Isaac ROS

### Combining Isaac ROS Perception with Nav2

Humanoid navigation benefits from combining Isaac ROS perception with Nav2:

```yaml
# Integration configuration for Isaac ROS + Nav2
isaac_ros_vslam:
  ros__parameters:
    use_sim_time: True
    enable_rectified_pose: True
    map_frame: "map"
    odom_frame: "odom"
    base_frame: "base_link"
    publish_odom_tf: True

humanoid_nav2:
  ros__parameters:
    use_sim_time: True
    global_frame: "map"
    robot_base_frame: "base_link"
    odom_topic: "odom"
    default_bt_xml_filename: "humanoid_navigate_w_replanning_and_recovery.xml"

# Costmap configuration that uses Isaac ROS perception
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
      global_frame: "odom"
      robot_base_frame: "base_link"
      use_sim_time: True
      rolling_window: true
      width: 10
      height: 10
      resolution: 0.05
      # Use Isaac ROS perception for obstacle detection
      observation_sources: scan pointcloud
      scan:
        topic: "/scan"
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
      pointcloud:
        topic: "/pointcloud"
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "PointCloud2"
        expected_update_rate: 0.5
        observation_persistence: 0.0
        inf_is_valid: True
        min_obstacle_height: 0.0
        obstacle_range: 5.0
        raytrace_range: 5.0
```

## Practical Exercise: Humanoid Navigation Setup

Set up Nav2 for humanoid robot navigation:

1. Configure costmaps for humanoid-specific constraints
2. Implement footstep planning plugin
3. Test navigation in simulation
4. Validate balance during path execution

### Launch File for Humanoid Navigation

```xml
<!-- Humanoid navigation launch file -->
<launch>
  <!-- Arguments -->
  <arg name="namespace" default=""/>
  <arg name="use_sim_time" default="true"/>
  <arg name="autostart" default="true"/>
  <arg name="params_file" default="$(find-pkg-share my_humanoid_navigation)/config/humanoid_nav2_params.yaml"/>
  <arg name="default_bt_xml_filename" default="humanoid_navigate_w_replanning_and_recovery.xml"/>
  <arg name="map_subscribe_transient_local" default="false"/>

  <!-- Map Server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server">
    <param name="yaml_filename" value="$(find-pkg-share my_humanoid_navigation)/maps/humanoid_world.yaml"/>
    <param name="frame_id" value="map"/>
    <param name="topic_name" value="map"/>
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>

  <!-- Lifecycle Manager -->
  <node pkg="nav2_lifecycle_manager" exec="lifecycle_manager" name="lifecycle_manager">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="autostart" value="$(var autostart)"/>
    <param name="node_names" value="[map_server, amcl, bt_navigator, controller_server, local_costmap, global_costmap, planner_server, recovery_server]"/>
  </node>

  <!-- AMCL -->
  <node pkg="nav2_amcl" exec="amcl" name="amcl">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="set_initial_pose" value="true"/>
    <param name="initial_pose.x" value="0.0"/>
    <param name="initial_pose.y" value="0.0"/>
    <param name="initial_pose.z" value="0.0"/>
    <param name="initial_pose.yaw" value="0.0"/>
  </node>

  <!-- Navigation Server -->
  <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="global_frame" value="map"/>
    <param name="robot_base_frame" value="base_link"/>
    <param name="odom_topic" value="odom"/>
    <param name="default_bt_xml_filename" value="$(var default_bt_xml_filename)"/>
    <param name="plugin_lib_names" value="[nav2_compute_path_to_pose_action, nav2_follow_path_action, nav2_back_up_action, nav2_spin_action, nav2_wait_action, humanoid_footstep_planner, humanoid_balance_checker]"/>
  </node>

  <!-- Controller Server -->
  <node pkg="nav2_controller" exec="controller_server" name="controller_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="controller_frequency" value="20.0"/>
    <param name="min_x_velocity_threshold" value="0.001"/>
    <param name="min_y_velocity_threshold" value="0.001"/>
    <param name="min_theta_velocity_threshold" value="0.001"/>
    <param name="progress_checker_plugin" value="progress_checker"/>
    <param name="goal_checker_plugin" value="goal_checker"/>
    <param name="controller_plugins" value="[FollowPath]"/>
    <param name="FollowPath.type" value="nav2_mppi_controller::MPPIController"/>
  </node>

  <!-- Local Costmap -->
  <node pkg="nav2_costmap_2d" exec="costmap_2d_node" name="local_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="global_frame" value="odom"/>
    <param name="robot_base_frame" value="base_link"/>
    <param name="update_frequency" value="5.0"/>
    <param name="publish_frequency" value="2.0"/>
    <param name="width" value="10.0"/>
    <param name="height" value="10.0"/>
    <param name="resolution" value="0.05"/>
    <param name="footprint" value="[[0.25, 0.15], [0.25, -0.15], [-0.25, -0.15], [-0.25, 0.15]]"/>  <!-- Humanoid footprint -->
  </node>

  <!-- Global Costmap -->
  <node pkg="nav2_costmap_2d" exec="costmap_2d_node" name="global_costmap" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="global_frame" value="map"/>
    <param name="robot_base_frame" value="base_link"/>
    <param name="update_frequency" value="1.0"/>
    <param name="publish_frequency" value="1.0"/>
    <param name="width" value="100.0"/>
    <param name="height" value="100.0"/>
    <param name="resolution" value="0.05"/>
    <param name="footprint" value="[[0.25, 0.15], [0.25, -0.15], [-0.25, -0.15], [-0.25, 0.15]]"/>  <!-- Humanoid footprint -->
  </node>

  <!-- Planner Server -->
  <node pkg="nav2_planner" exec="planner_server" name="planner_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
    <param name="planner_plugins" value="[GridBased]"/>
    <param name="GridBased.type" value="nav2_navfn_planner::NavfnPlanner"/>
  </node>

  <!-- Recovery Server -->
  <node pkg="nav2_recoveries" exec="recoveries_server" name="recovery_server" output="screen">
    <param name="use_sim_time" value="$(var use_sim_time)"/>
  </node>
</launch>
```

## Performance and Safety Considerations

### Stability Monitoring

Humanoid navigation requires continuous stability monitoring:

```python
# Stability monitoring for humanoid navigation
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class StabilityMonitor(Node):
    def __init__(self):
        super().__init__('stability_monitor')

        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, 'current_pose', self.pose_callback, 10)

        self.stability_pub = self.create_publisher(
            Bool, 'is_stable', 10)

        self.balance_threshold = 15.0  # degrees
        self.com_height = 0.8  # meters

    def imu_callback(self, msg):
        # Calculate tilt angles from IMU
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # Check if tilt exceeds safe limits
        tilt_angle = max(abs(roll), abs(pitch))

        is_stable_msg = Bool()
        is_stable_msg.data = tilt_angle < self.balance_threshold

        self.stability_pub.publish(is_stable_msg)

        if not is_stable_msg.data:
            self.get_logger().warn(f'Balance threshold exceeded: {tilt_angle:.2f}°')

    def pose_callback(self, msg):
        # Monitor pose for balance considerations
        pass

    def quaternion_to_euler(self, x, y, z, w):
        import math
        # Convert quaternion to Euler angles
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
```

## Summary

In this chapter, you learned how to adapt the Nav2 stack for humanoid robots, considering unique challenges like bipedal locomotion, balance constraints, and step-aware navigation. You explored footstep planning, balance-aware path planning, and humanoid-specific navigation behaviors. This completes Module 3 on The AI-Robot Brain (NVIDIA Isaac™), providing comprehensive coverage of Isaac Sim, Isaac ROS, and Nav2 integration for humanoid robotics applications.