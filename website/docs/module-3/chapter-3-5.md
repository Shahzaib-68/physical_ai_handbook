---
sidebar_position: 15
slug: /module-3/chapter-3-5
title: Chapter 3.5 - Nav2 - Path Planning for Bipedal Movement
---

# Chapter 3.5: Nav2 - Path Planning for Bipedal Movement

## Overview

Navigation 2 (Nav2) is the ROS 2 navigation stack designed for mobile robots. While traditionally focused on wheeled robots, this chapter explores how to adapt and extend Nav2 for bipedal robots with complex locomotion patterns. We'll examine specialized path planning algorithms, costmap configurations, and control systems tailored for humanoid robots.

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the Nav2 architecture and its components
- Configure Nav2 for bipedal robot navigation
- Implement custom path planners suitable for legged locomotion
- Adapt costmaps for bipedal movement constraints
- Integrate whole-body motion planning with Nav2
- Evaluate and tune navigation performance for humanoid robots
- Design fallback behaviors for complex terrain navigation

## Introduction to Nav2

Navigation 2 (Nav2) is the official ROS 2 navigation stack that provides a complete solution for robot navigation. It includes components for path planning, obstacle avoidance, and motion execution.

### Key Components of Nav2:
1. **Navigation Server**: Main orchestration node
2. **Path Planner**: Global and local path planning
3. **Controller**: Trajectory generation and execution
4. **Recovery Behaviors**: Fallback strategies for navigation failures
5. **Costmap 2D**: Dynamic obstacle representation
6. **Behavior Trees**: Task planning and behavior management

### Nav2 Architecture for Bipedal Robots:
```
[Goal Specification] → [Global Planner] → [Path for Bipedal Robot] → [Local Planner] → [Trajectory]
                              ↓                                        ↓                   ↓
                      [Costmaps for Legs]                    [Bipedal Controller]    [Leg Control]
```

## Nav2 for Bipedal Robots

### Challenges with Bipedal Navigation:
- **Complex Dynamics**: Bipedal robots have more complex kinematics/dynamics than wheeled robots
- **Step Planning**: Need to plan where to place each foot
- **Balance Constraints**: Must maintain balance during navigation
- **Terrain Adaptation**: Different gaits needed for different terrains
- **Z-Motion**: Height changes require special consideration

### Customizations Needed for Bipedal Robots:
1. **Specialized Costmaps**: Account for leg clearance and foot placement
2. **Gait-Specific Planners**: Plan paths considering specific walking patterns
3. **Balance-Aware Controllers**: Ensure stable motion execution
4. **Terrain Classification**: Different locomotion modes for different terrains

## Nav2 Configuration for Bipedal Robots

### Basic Nav2 Launch File:
```xml
<!-- bipedal_nav2.launch.py -->
<launch>
  <!-- Map server -->
  <node pkg="nav2_map_server" exec="map_server" name="map_server">
    <param name="yaml_filename" value="/path/to/map.yaml" />
    <param name="topic" value="map" />
    <param name="frame_id" value="map" />
    <param name="output" value="screen" />
  </node>
  
  <!-- Local and Global Costmaps -->
  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="local_costmap">
    <param name="use_sim_time" value="True" />
    <param name="global_frame" value="odom" />
    <param name="robot_base_frame" value="base_link" />
    <param name="update_frequency" value="5.0" />
    <param name="publish_frequency" value="2.0" />
    <param name="width" value="10.0" />
    <param name="height" value="10.0" />
    <param name="resolution" value="0.05" />
    <param name="footprint" value="[[0.5, 0.3], [0.5, -0.3], [-0.5, -0.3], [-0.5, 0.3]]" />
    <param name="plugins" value="[\"static_layer\", \"obstacle_layer\", \"inflation_layer\"]" />
  </node>

  <node pkg="nav2_costmap_2d" exec="nav2_costmap_2d" name="global_costmap">
    <param name="use_sim_time" value="True" />
    <param name="global_frame" value="map" />
    <param name="robot_base_frame" value="base_link" />
    <param name="update_frequency" value="1.0" />
    <param name="static_map" value="true" />
    <param name="width" value="200.0" />
    <param name="height" value="200.0" />
    <param name="resolution" value="0.1" />
    <param name="footprint" value="[[0.5, 0.3], [0.5, -0.3], [-0.5, -0.3], [-0.5, 0.3]]" />
    <param name="plugins" value="[\"static_layer\", \"obstacle_layer\", \"inflation_layer\"]" />
  </node>

  <!-- Global Planner for Bipedal Movement -->
  <node pkg="nav2_navfn_planner" exec="navfn_planner" name="global_planner">
    <param name="use_sim_time" value="True" />
    <param name="allow_unknown" value="true" />
    <param name="use_astar" value="false" />
    <param name="planner_name" value="GridBased" />
    <param name="plugin_names" value="['GridBased']" />
    <param name="GridBased.type" value="nav2_navfn_planner/NavfnPlanner" />
  </node>

  <!-- Local Planner for Bipedal Movement -->
  <node pkg="nav2_simple_commander" exec="nav2_simple_commander" name="simple_commander">
    <param name="use_sim_time" value="True" />
  </node>

  <!-- Navigation Server -->
  <node pkg="nav2_bt_navigator" exec="bt_navigator" name="bt_navigator">
    <param name="use_sim_time" value="True" />
    <param name="bt_loop_duration" value="10" />
    <param name="default_server_timeout" value="20" />
    <param name="plugin_lib_names" value="['navigate_to_pose', 'follow_path', 'spin', 'backup', 'wait']" />
  </node>
</launch>
```

## Custom Path Planners for Bipedal Robots

### Standard Global Planner Limitations:
Traditional path planners (e.g., A*, Dijkstra) plan for circular or rectangular robots, which doesn't account for the specific constraints of bipedal locomotion.

### Bipedal-Aware Path Planner:
```cpp
// C++ example of a custom global planner for bipedal navigation
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "angles/angles.h"
#include <string>
#include <memory>

class BipedalGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
    BipedalGlobalPlanner() = default;
    ~BipedalGlobalPlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        node_ = parent.lock();
        name_ = name;
        tf_ = tf;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        
        // Additional configuration for bipedal-specific parameters
        node_->declare_parameter(name_ + ".max_step_height", 0.15);  // Maximum step height
        node_->declare_parameter(name_ + ".foot_size", 0.25);       // Approximate foot size
        node_->declare_parameter(name_ + ".max_inclination", 15.0); // Maximum slope in degrees
        
        max_step_height_ = node_->get_parameter(name_ + ".max_step_height").as_double();
        foot_size_ = node_->get_parameter(name_ + ".foot_size").as_double();
        max_inclination_ = node_->get_parameter(name_ + ".max_inclination").as_double();
    }

    void cleanup() override
    {
        RCLCPP_INFO(node_->get_logger(), "Cleaning up bipedal global planner");
    }

    void activate() override
    {
        RCLCPP_INFO(node_->get_logger(), "Activating bipedal global planner");
    }

    void deactivate() override
    {
        RCLCPP_INFO(node_->get_logger(), "Deactivating bipedal global planner");
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();
        
        // Check if goal is valid
        if (!isGoalValid(start, goal)) {
            RCLCPP_WARN(node_->get_logger(), "Goal is not valid for bipedal navigation");
            return path;  // Return empty path
        }
        
        // Implement bipedal-aware path planning algorithm
        // This is a simplified example - in practice, this would involve:
        // 1. Footstep planning
        // 2. Balance-aware path optimization
        // 3. Terrain analysis for appropriate gaits
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        if (computeBipedalPath(start, goal, plan)) {
            path.poses = plan;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Could not compute path for bipedal robot");
        }
        
        return path;
    }

private:
    bool isGoalValid(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal)
    {
        // Check if the goal is reachable considering bipedal constraints
        // - Is the goal on walkable terrain?
        // - Is there sufficient clearance for leg movement?
        // - Is the ground slope within acceptable limits?
        
        // Convert goal coordinates to map indices
        unsigned int goal_x, goal_y;
        if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
            return false;  // Goal is outside map
        }
        
        // Check slope constraints
        unsigned char cost = costmap_->getCost(goal_x, goal_y);
        if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
            return false;  // Goal is in obstacle
        }
        
        // Additional checks for bipedal constraints would go here
        return true;
    }
    
    bool computeBipedalPath(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::vector<geometry_msgs::msg::PoseStamped> & plan)
    {
        // In a real implementation, this would use a specialized path planning
        // algorithm that considers:
        // - Foot placement constraints
        // - Balance requirements
        // - Different locomotion modes (walking, stepping, climbing)
        
        // For this example, we'll use a simplified approach
        // that creates a path considering foot size
        
        // Calculate path using a modified A* that considers foot print
        // This would call a more sophisticated planning algorithm
        return computeSimplePath(start, goal, plan);
    }
    
    bool computeSimplePath(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::vector<geometry_msgs::msg::PoseStamped> & plan)
    {
        // Create a simple straight-line path with intermediate waypoints
        // considering foot placement
        geometry_msgs::msg::PoseStamped current_pose = start;
        double dx = goal.pose.position.x - start.pose.position.x;
        double dy = goal.pose.position.y - start.pose.position.y;
        double dist = sqrt(dx*dx + dy*dy);
        double step_size = foot_size_ * 0.5;  // Stepsize based on foot size
        
        int num_steps = static_cast<int>(dist / step_size);
        
        for (int i = 0; i <= num_steps; i++) {
            geometry_msgs::msg::PoseStamped waypoint = start;
            double ratio = static_cast<double>(i) / num_steps;
            
            waypoint.pose.position.x = start.pose.position.x + dx * ratio;
            waypoint.pose.position.y = start.pose.position.y + dy * ratio;
            
            // Ensure orientation is maintained towards goal
            double angle = atan2(dy, dx);
            waypoint.pose.orientation.z = sin(angle / 2.0);
            waypoint.pose.orientation.w = cos(angle / 2.0);
            
            plan.push_back(waypoint);
        }
        
        return true;
    }
    
    bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
    {
        if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
            return false;
        }
        
        mx = static_cast<unsigned int>((wx - costmap_->getOriginX()) / costmap_->getResolution());
        my = static_cast<unsigned int>((wy - costmap_->getOriginY()) / costmap_->getResolution());
        
        if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
            return true;
        }
        
        return false;
    }
    
    // Member variables
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_frame_;
    
    // Bipedal-specific parameters
    double max_step_height_;
    double foot_size_;
    double max_inclination_;
};
```

## Costmap Configuration for Bipedal Robots

### Standard Costmap Limitations:
Standard costmaps are designed for wheeled robots and don't consider leg clearance, foot placement, or balance constraints.

### Extended Costmap for Bipedal Navigation:

```yaml
# bipedal_costmap_params.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 10.0
  height: 10.0
  resolution: 0.05
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
  
  # Bipedal-specific parameters
  foot_size: [0.25, 0.15]  # Foot dimensions (length, width)
  leg_clearance: 0.1       # Minimum clearance under legs
  max_step_height: 0.15    # Maximum traversable step height
  balance_radius: 0.3      # Radius for balance checks
  
obstacle_layer:
  enabled: true
  observation_sources: scan
  scan:
    topic: /scan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    data_type: LaserScan
    obstacle_range: 3.0
    raytrace_range: 4.0

inflation_layer:
  enabled: true
  cost_scaling_factor: 5.0
  inflation_radius: 0.5  # Increased for bipedal safety margin
  inflate_unknown: false
```

## Bipedal-Specific Controllers

### Challenges with Standard Controllers:
- Standard controllers assume continuous motion
- Don't account for discrete foot placements
- Don't consider balance during transitions

### Custom Bipedal Controller Node:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import numpy as np
from scipy.spatial.transform import Rotation as R


class BipedalController(Node):
    def __init__(self):
        super().__init__('bipedal_controller')
        
        # Subscribers
        self.path_sub = self.create_subscription(
            Path, '/plan', self.path_callback, 10)
        self.odom_sub = self.create_subscription(
            String, '/robot_state', self.odom_callback, 10)  # Simplified state
        
        # Publishers  
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/bipedal_status', 10)
        
        # Parameters
        self.declare_parameter('step_size', 0.3)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.5)
        
        self.step_size = self.get_parameter('step_size').value
        self.step_height = self.get_parameter('step_height').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Internal state
        self.current_path = None
        self.current_waypoint_idx = 0
        self.robot_pose = None
        self.is_moving = False
        self.current_gait = "walking"
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Bipedal Controller Initialized")
    
    def path_callback(self, path_msg):
        self.get_logger().info(f"Received path with {len(path_msg.poses)} waypoints")
        self.current_path = path_msg.poses
        self.current_waypoint_idx = 0
        self.is_moving = True
    
    def odom_callback(self, state_msg):
        # In a real implementation, this would update robot pose
        # For this example, we'll just acknowledge the state
        pass
    
    def control_loop(self):
        if not self.is_moving or not self.current_path:
            return
        
        if self.current_waypoint_idx >= len(self.current_path):
            # Reached goal
            self.stop_robot()
            self.publish_status("Goal reached")
            return
        
        # Get current target waypoint
        target_pose = self.current_path[self.current_waypoint_idx]
        
        # Calculate distance to target
        if self.robot_pose:
            dx = target_pose.pose.position.x - self.robot_pose.position.x
            dy = target_pose.pose.position.y - self.robot_pose.position.y
            distance = np.sqrt(dx*dx + dy*dy)
            
            # Check if close enough to target
            if distance < 0.1:  # 10cm threshold
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx >= len(self.current_path):
                    self.stop_robot()
                    self.publish_status("Goal reached")
                    return
                # Get next target
                target_pose = self.current_path[self.current_waypoint_idx]
        
        # Compute control commands considering bipedal constraints
        cmd_vel = self.compute_bipedal_control(target_pose)
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
        
        # Update status
        self.publish_status(f"Moving to waypoint {self.current_waypoint_idx}")

    def compute_bipedal_control(self, target_pose):
        cmd = Twist()
        
        if not self.robot_pose:
            return cmd  # Can't compute without pose
        
        # Calculate direction to target
        dx = target_pose.pose.position.x - self.robot_pose.position.x
        dy = target_pose.pose.position.y - self.robot_pose.position.y
        target_angle = np.arctan2(dy, dx)
        
        # Calculate robot's current orientation
        current_angle = self.get_yaw_from_quaternion(self.robot_pose.orientation)
        
        # Calculate angle error
        angle_error = target_angle - current_angle
        # Normalize angle to [-π, π]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        # PID-like control for angular velocity
        angular_kp = 1.0
        cmd.angular.z = np.clip(angular_kp * angle_error, 
                                -self.max_angular_speed, self.max_angular_speed)
        
        # Linear velocity based on distance and angle alignment
        distance = np.sqrt(dx*dx + dy*dy)
        
        # Only move forward if somewhat aligned with target direction
        if abs(angle_error) < 0.5:  # 0.5 radians ~ 28.6 degrees
            cmd.linear.x = min(self.max_linear_speed, distance * 1.0)
        
        # Simulate discrete stepping by limiting maximum speed
        cmd.linear.x = min(cmd.linear.x, self.step_size * 2)  # 2 steps per sec max
        
        return cmd

    def get_yaw_from_quaternion(self, quat):
        # Convert quaternion to yaw angle
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)
    
    def stop_robot(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.is_moving = False
    
    def publish_status(self, status_msg):
        status = String()
        status.data = status_msg
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    bipedal_controller = BipedalController()
    
    try:
        rclpy.spin(bipedal_controller)
    except KeyboardInterrupt:
        pass
    finally:
        bipedal_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Whole-Body Motion Planning Integration

### Combining Nav2 with Whole-Body Planners:

```python
class WholeBodyMotionIntegrator(Node):
    def __init__(self):
        super().__init__('whole_body_motion_integrator')
        
        # Subscribe to Nav2 path
        self.nav_path_sub = self.create_subscription(
            Path, '/plan', self.nav_path_callback, 10)
        
        # Subscribe to high-level commands
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Publishers for whole-body commands
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Service clients for whole-body planner
        self.wb_planner_client = self.create_client(
            ComputeJointTrajectory, 'compute_whole_body_trajectory')
        
        # Internal state
        self.nav_path = None
        self.current_goal = None
        self.whole_body_trajectory = None
        
        # Timer for trajectory execution
        self.trajectory_timer = self.create_timer(0.01, self.execute_trajectory)
        
        self.get_logger().info("Whole Body Motion Integrator Initialized")
    
    def nav_path_callback(self, path_msg):
        self.nav_path = path_msg
        # Convert navigation path to whole-body motion plan
        self.compute_whole_body_trajectory()
    
    def compute_whole_body_trajectory(self):
        if not self.nav_path or not self.wb_planner_client.service_is_ready():
            return
        
        # Create service request
        request = ComputeJointTrajectory.Request()
        request.path = self.nav_path
        request.robot_config = self.get_current_robot_config()
        
        # Call whole-body planner service
        future = self.wb_planner_client.call_async(request)
        future.add_done_callback(self.whole_body_trajectory_callback)
    
    def whole_body_trajectory_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.whole_body_trajectory = response.trajectory
                self.execute_whole_body_motion()
            else:
                self.get_logger().error("Whole-body trajectory computation failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def execute_whole_body_motion(self):
        # Execute the computed whole-body trajectory
        if self.whole_body_trajectory:
            self.joint_trajectory_pub.publish(self.whole_body_trajectory)
    
    def execute_trajectory(self):
        # Execute trajectory point-by-point
        pass
```

## Terrain-Aware Navigation

### Adaptive Navigation Based on Terrain:

```python
class TerrainAwareNavigator:
    def __init__(self, node):
        self.node = node
        
        # Subscribe to terrain classification
        self.terrain_sub = node.create_subscription(
            String, '/terrain_classification', self.terrain_callback, 10)
        
        # Current terrain type
        self.current_terrain = "flat"
        
        # Terrain-specific parameters
        self.terrain_params = {
            "flat": {"max_speed": 0.8, "step_size": 0.3, "gait": "normal_walk"},
            "uneven": {"max_speed": 0.4, "step_size": 0.2, "gait": "careful_walk"},
            "stairs": {"max_speed": 0.2, "step_size": 0.15, "gait": "step_climbing"},
            "slope": {"max_speed": 0.3, "step_size": 0.25, "gait": "slope_navigation"}
        }
    
    def terrain_callback(self, terrain_msg):
        self.current_terrain = terrain_msg.data
        self.update_navigation_parameters()
    
    def update_navigation_parameters(self):
        params = self.terrain_params.get(self.current_terrain, 
                                       self.terrain_params["flat"])
        
        # Update controller parameters based on terrain
        self.node.get_logger().info(f"Updating for terrain: {self.current_terrain}")
        # In practice, this would dynamically reconfigure Nav2 parameters
```

## Performance Evaluation

### Metrics for Bipedal Navigation:

```python
class BipedalNavigationMetrics:
    def __init__(self):
        self.metrics = {
            'path_efficiency': 0.0,     # Path length vs. straight-line distance
            'balance_stability': 0.0,   # ZMP (Zero Moment Point) stability 
            'gait_consistency': 0.0,    # Consistency of stepping pattern
            'navigation_success': 0.0,  # Success rate of reaching goals
            'computation_time': 0.0,    # Average planning time
            'energy_efficiency': 0.0    # Energy per unit distance
        }
        
        self.trial_count = 0
        self.success_count = 0
    
    def evaluate_path(self, planned_path, actual_path, goal_reached):
        # Calculate path efficiency
        straight_line_distance = self.calculate_straight_line_distance(actual_path)
        actual_distance = self.calculate_path_distance(actual_path)
        
        self.metrics['path_efficiency'] = straight_line_distance / actual_distance if actual_distance > 0 else 0
        
        # Update success statistics
        if goal_reached:
            self.success_count += 1
        self.trial_count += 1
        
        self.metrics['navigation_success'] = self.success_count / self.trial_count if self.trial_count > 0 else 0
    
    def calculate_straight_line_distance(self, path):
        if len(path.poses) < 2:
            return 0.0
        start = path.poses[0]
        end = path.poses[-1]
        dx = end.pose.position.x - start.pose.position.x
        dy = end.pose.position.y - start.pose.position.y
        return np.sqrt(dx*dx + dy*dy)
    
    def calculate_path_distance(self, path):
        total_distance = 0.0
        for i in range(1, len(path.poses)):
            prev_pose = path.poses[i-1].pose.position
            curr_pose = path.poses[i].pose.position
            dx = curr_pose.x - prev_pose.x
            dy = curr_pose.y - prev_pose.y
            total_distance += np.sqrt(dx*dx + dy*dy)
        return total_distance
    
    def generate_evaluation_report(self):
        report = f"""
        Bipedal Navigation Performance Report:
        - Path Efficiency: {self.metrics['path_efficiency']:.2f}
        - Balance Stability: {self.metrics['balance_stability']:.2f}
        - Gait Consistency: {self.metrics['gait_consistency']:.2f}
        - Success Rate: {self.metrics['navigation_success']:.2f}
        - Avg Computation Time: {self.metrics['computation_time']:.2f} ms
        - Energy Efficiency: {self.metrics['energy_efficiency']:.2f} J/m
        """
        return report
```

## Best Practices for Bipedal Navigation

### 1. Parameter Tuning:
- Start with conservative parameters and gradually increase
- Test on various terrain types
- Monitor robot balance metrics continuously
- Implement real-time parameter adaptation

### 2. Safety Considerations:
- Implement emergency stops for balance loss
- Use conservative obstacle inflation
- Plan with stability margins
- Monitor ZMP (Zero Moment Point) in real-time

### 3. System Integration:
- Properly calibrate all sensors
- Ensure consistent coordinate frames
- Implement proper error handling and recovery
- Provide feedback to operators

### 4. Testing Strategy:
- Start with simple, known environments
- Gradually increase complexity
- Test edge cases and failure scenarios
- Validate in simulation before real-world deployment

## Summary

This chapter explored the adaptation of the Nav2 navigation stack for bipedal robots. We covered specialized path planners, costmap configurations, and controllers designed specifically for legged locomotion patterns.

Key takeaways include:
- Bipedal robots require specialized navigation approaches due to their complex dynamics and balance requirements
- Standard Nav2 components need modification to account for foot placement and step planning
- Integration with whole-body motion planners is essential for stable locomotion
- Terrain-aware navigation is crucial for effective bipedal locomotion

Successfully implementing navigation for bipedal robots requires a deep understanding of both general navigation principles and the specific constraints of legged locomotion.

## Exercises

1. Configure a Nav2 instance for a simulated bipedal robot in Gazebo.
2. Implement a custom path planner that considers foot placement constraints.
3. Adapt costmap parameters for bipedal robot foot size and balance requirements.
4. Design and test different locomotion modes (walking, stepping) for various terrains.
5. Evaluate the performance of your bipedal navigation system in different scenarios.