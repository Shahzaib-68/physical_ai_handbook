---
sidebar_position: 5
slug: /module-1/chapter-1-5
title: Chapter 1.5 - Building Your First ROS 2 Package
---

# Chapter 1.5: Building Your First ROS 2 Package

## Overview

A ROS 2 package is the fundamental building block for organizing code, resources, and configurations in ROS. Creating well-structured packages is essential for developing robust robotic applications. This chapter guides you through creating your first ROS 2 package with a focus on developing components for humanoid robotics applications.

## Learning Objectives

By the end of this chapter, students will be able to:
- Create a new ROS 2 package with proper structure
- Implement nodes in both C++ and Python
- Configure package dependencies and build systems
- Create launch files for running multiple nodes
- Add custom message and service definitions
- Build and test the developed ROS 2 package

## Package Structure

A typical ROS 2 package follows this structure:

```
my_robot_package/
├── CMakeLists.txt          # Build configuration for C++
├── package.xml             # Package metadata and dependencies
├── src/                    # Source code files
│   ├── cpp_node.cpp
│   └── python_node.py
├── include/                # Header files (for C++)
├── launch/                 # Launch files
│   └── robot_launch.py
├── config/                 # Configuration files
├── msg/                    # Custom message definitions
├── srv/                    # Custom service definitions
├── action/                 # Custom action definitions
└── test/                   # Test files
```

## Creating a New Package

Use the `ros2 pkg create` command to create a new package with the proper structure:

```bash
ros2 pkg create --build-type ament_cmake my_robot_package
```

For packages that will contain both C++ and Python nodes:

```bash
ros2 pkg create --build-type ament_cmake --dependencies rclcpp rclpy std_msgs sensor_msgs geometry_msgs my_robot_msgs my_robot_services my_robot_actions my_robot_description my_robot_control
```

## Package.xml Configuration

The `package.xml` file contains metadata about your package and its dependencies:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Package for controlling my humanoid robot</description>
  <maintainer email="maintainer@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <depend>message_runtime</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Creating a C++ Node

Create a C++ node in `src/cpp_robot_control.cpp`:

```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class RobotController : public rclcpp::Node
{
public:
    RobotController()
    : Node("robot_controller"), 
      joint_position_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}  // Initialize 6 joint positions
    {
        // Create publisher for joint commands
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_commands", 10);
        
        // Create subscriber for velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, 
            std::bind(&RobotController::cmd_vel_callback, this, std::placeholders::_1));
        
        // Create timer for periodic joint updates
        timer_ = this->create_wall_timer(
            50ms, std::bind(&RobotController::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Robot Controller Node Started");
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Process velocity command and update internal state
        RCLCPP_INFO(this->get_logger(), 
            "Received linear: [%f], angular: [%f]", 
            msg->linear.x, msg->angular.z);
            
        // Store the command for use in timer callback
        last_cmd_vel_ = *msg;
    }
    
    void timer_callback()
    {
        // Update joint positions based on stored command
        update_joints();
        
        // Publish joint state
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        msg.position = joint_position_;
        
        joint_pub_->publish(msg);
    }
    
    void update_joints()
    {
        // Simple simulation of how velocity commands affect joint positions
        // In a real robot, this would implement inverse kinematics or joint control
        
        for (auto& pos : joint_position_) {
            pos += 0.01;  // Simulate small movement
            if (pos > 3.14) pos = -3.14;  // Wrap around
        }
    }
    
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist last_cmd_vel_;
    std::vector<double> joint_position_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}
```

## Creating a Python Node

Create a Python node in `my_robot_package/my_robot_package/python_robot_controller.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import math

class PythonRobotController(Node):
    def __init__(self):
        super().__init__('python_robot_controller')
        
        # Create publisher for status updates
        self.status_publisher = self.create_publisher(String, '/robot_status', 10)
        
        # Create subscriber for joint states
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Create publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(
            JointState, '/joint_commands', 10)
        
        # Timer for status updates
        self.timer = self.create_timer(1.0, self.status_timer_callback)
        
        # Internal state
        self.joint_positions = [0.0] * 6
        self.robot_status = "Idle"
        
        self.get_logger().info("Python Robot Controller Node Started")

    def joint_state_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.position}')
        self.joint_positions = list(msg.position)
        
        # Update robot status based on joint positions
        if any(abs(pos) > 2.0 for pos in self.joint_positions):
            self.robot_status = "Active"
        else:
            self.robot_status = "Idle"

    def status_timer_callback(self):
        msg = String()
        msg.data = f"Status: {self.robot_status}, Joints: {self.joint_positions}"
        self.status_publisher.publish(msg)
        
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    
    controller = PythonRobotController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## CMakeLists.txt Configuration

Configure the build system in `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable for C++ node
add_executable(robot_controller src/cpp_robot_control.cpp)
ament_target_dependencies(robot_controller 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs)

# Install executables
install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME})

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  my_robot_package/python_robot_controller.py
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

## Creating Launch Files

Create a launch file in `launch/robot_launch.py` to run multiple nodes together:

```python
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # Get launch configuration
    namespace = LaunchConfiguration('namespace')
    
    # Define robot controller node
    robot_controller_node = Node(
        package='my_robot_package',
        executable='robot_controller',
        name='robot_controller',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True}  # Use simulation time if available
        ],
        remappings=[
            ('/joint_commands', '/my_robot/joint_commands'),
            ('/cmd_vel', '/my_robot/cmd_vel')
        ],
        output='screen'
    )
    
    # Define Python controller node
    python_controller_node = Node(
        package='my_robot_package',
        executable='python_robot_controller.py',
        name='python_robot_controller',
        namespace=namespace,
        parameters=[
            {'use_sim_time': True}
        ],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        namespace_arg,
        robot_controller_node,
        python_controller_node
    ])
```

## Adding Custom Messages

Create a custom message in `msg/RobotStatus.msg`:

```
string status
float64[] joint_positions
float64 battery_level
bool is_charging
```

Update `CMakeLists.txt` to include custom messages:

```cmake
find_package(rosidl_default_generators REQUIRED)

# Define custom messages
set(msg_files
  "msg/RobotStatus.msg"
)

# Generate message headers
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# Add dependency to executable
ament_target_dependencies(robot_controller 
  rclcpp 
  std_msgs 
  geometry_msgs 
  sensor_msgs
  ${PROJECT_NAME})  # Add this for custom messages
```

## Building the Package

To build your ROS 2 package:

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash  # Replace with your distro name

# Navigate to your workspace
cd ~/ros2_ws

# Build the specific package
colcon build --packages-select my_robot_package

# Or build all packages
colcon build

# Source the workspace
source install/setup.bash
```

## Testing Your Package

Test your package functionality:

1. **Run the nodes individually**:
```bash
# For C++ node
ros2 run my_robot_package robot_controller

# For Python node
ros2 run my_robot_package python_robot_controller.py
```

2. **Run with launch file**:
```bash
ros2 launch my_robot_package robot_launch.py
```

3. **Check active nodes**:
```bash
ros2 node list
```

4. **Check topics**:
```bash
ros2 topic list
```

5. **Publish test messages**:
```bash
# Send a velocity command to the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}, angular: {z: 0.5}}'
```

## Best Practices

### 1. Package Naming
- Use lowercase with underscores
- Be descriptive but concise
- Follow ROS naming conventions

### 2. Code Organization
- Separate concerns into different nodes
- Use appropriate namespaces
- Implement proper error handling

### 3. Parameter Configuration
- Use parameters for configurable values
- Document all parameters
- Provide sensible defaults

### 4. Logging
- Use appropriate log levels (info, warn, error)
- Include relevant context in log messages
- Avoid excessive logging in performance-critical loops

### 5. Documentation
- Include README.md files
- Document interfaces clearly
- Add usage examples

## Summary

This chapter covered the complete workflow for creating a ROS 2 package for humanoid robotics applications:
- Proper package structure and organization
- Creating nodes in both C++ and Python
- Configuring build systems and dependencies
- Using launch files for complex setups
- Adding custom message types
- Building and testing the package

A well-structured ROS 2 package is the foundation for any robotic application and enables modular, maintainable code that can be easily integrated with other systems.

## Exercises

1. Create a new ROS 2 package called `humanoid_controller` with nodes that simulate controlling a humanoid robot's joints.
2. Add custom message definitions for humanoid robot states and commands.
3. Create launch files that start multiple nodes simulating different aspects of a humanoid robot.
4. Implement proper parameter configuration for your nodes.
5. Add unit tests for your nodes using ROS 2's testing framework.