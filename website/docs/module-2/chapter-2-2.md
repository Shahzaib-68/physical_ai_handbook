---
sidebar_position: 7
slug: /module-2/chapter-2-2
title: Chapter 2.2 - Gazebo Environment Setup and Basics
---

# Chapter 2.2: Gazebo Environment Setup and Basics

## Overview

Gazebo is a powerful, open-source robotics simulator that provides high-fidelity physics simulation and rendering capabilities. This chapter covers the fundamentals of setting up and using Gazebo for robotics applications, with a focus on humanoid robot simulation. We'll explore installation, basic usage, and the core components of the Gazebo environment.

## Learning Objectives

By the end of this chapter, students will be able to:
- Install and configure Gazebo for robotics simulation
- Launch and navigate the Gazebo interface
- Create and modify basic simulation environments
- Spawn and manipulate robots in Gazebo
- Understand gazebo's plugin architecture
- Integrate Gazebo with ROS 2 for robot control

## Installing Gazebo

Gazebo is typically installed as part of a ROS distribution. For ROS 2 distributions:

### Ubuntu Installation:
```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```

### Windows Installation:
```bash
# Using Chocolatey
choco install gazebo

# Or install from source following the official documentation
```

### Docker Approach:
For consistent environments across platforms:
```bash
docker pull osrf/gazebo:gz-sim-harmonic
docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix osrf/gazebo:gz-sim-harmonic
```

## Gazebo Architecture Overview

Gazebo consists of several key components:

### 1. Physics Engine
- Handles collision detection and dynamics simulation
- Supports ODE, Bullet, Simbody, and DART engines
- Configured in the world file

### 2. Rendering Engine
- Provides 3D visualization
- Options include Ogre, OpenGL, or headless rendering
- Supports realistic lighting and materials

### 3. Communication Interface
- Uses transport layer for internal communication
- Supports plugins for external communication
- Integrates with ROS/ROS 2 via gazebo_ros_pkgs

## Launching Gazebo

### Basic Launch:
```bash
gazebo
```

### Launch with a World File:
```bash
gazebo /path/to/world_file.world
```

### Launch from ROS 2:
```bash
# Using ROS 2 launch files
ros2 launch gazebo_ros empty_world.launch.py
```

## Gazebo Interface Navigation

The Gazebo interface consists of:

### 1. Main Simulation Window
- 3D visualization of the simulated world
- Can be rotated, panned, and zoomed
- Interactive manipulation of objects

### 2. Toolbar
- Simulation controls (play, pause, reset)
- Object insertion tools
- Camera controls

### 3. Scene Graph
- Hierarchical view of simulation objects
- Enables object selection and manipulation
- Shows model relationships

### 4. Layers Window
- Controls rendering of different visual elements
- Can hide/show specific entities
- Adjusts visual properties

## Creating Basic Worlds

### World File Structure:
Gazebo worlds are defined in SDF (Simulation Description Format) format:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="default">
    <!-- Include models from Gazebo model database -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Define static objects -->
    <model name="table">
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1.0 1.0 0.05</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1.0 1.0 0.05</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.2 0.1 1</ambient>
            <diffuse>0.8 0.2 0.1 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyy>1.0</iyy>
            <iyz>0.0</iyz>
            <izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>

    <!-- Physics parameters -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000.0</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

### Loading Models:
Models can be added to the simulation in several ways:

1. **From the Model Database**:
   - Use the "Insert" tab to browse available models
   - Models are stored in `~/.gazebo/models/` or `/usr/share/gazebo-*/models/`

2. **From URDF Files**:
   ```bash
   # Spawn robot from URDF
   ros2 run gazebo_ros spawn_entity.py -file /path/to/robot.urdf -entity my_robot
   ```

3. **Programmatically**:
   ```python
   import rclpy
   from gazebo_msgs.srv import SpawnEntity
   
   # Service client to spawn entities
   spawn_cli = node.create_client(SpawnEntity, '/spawn_entity')
   ```

## Basic Simulation Operations

### 1. Controlling Simulation Flow
- **Play/Pause**: Control simulation time
- **Step**: Advance simulation by a single time step
- **Reset**: Reset simulation to initial state

### 2. Object Manipulation
- **Translate**: Move objects in 3D space
- **Rotate**: Change object orientation
- **Scale**: Resize objects (with some limitations)

### 3. Camera Control
- **Orbit**: Rotate camera around a target
- **Pan**: Move camera parallel to the viewing plane
- **Zoom**: Adjust camera distance

## Working with Robots in Gazebo

### Spawning a Robot:
```bash
# Using ROS 2
ros2 run gazebo_ros spawn_entity.py -file $(ros2 pkg prefix my_robot_description)/share/my_robot_description/urdf/my_robot.urdf -entity my_robot -x 0 -y 0 -z 1
```

### Basic Robot Control:
For joint control, you'll need ROS 2 controllers:

1. **Joint State Publisher**:
   ```xml
   <gazebo>
     <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
       <joint_name>joint1</joint_name>
       <joint_name>joint2</joint_name>
     </plugin>
   </gazebo>
   ```

2. **Effort/Position Controllers**:
   ```xml
   <gazebo>
     <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
       <robotNamespace>/my_robot</robotNamespace>
     </plugin>
   </gazebo>
   ```

## Gazebo Plugins

Gazebo supports plugins to extend functionality:

### 1. Sensor Plugins
- `libgazebo_ros_camera.so`: Camera sensors
- `libgazebo_ros_imu.so`: IMU sensors
- `libgazebo_ros_laser.so`: LiDAR sensors

### 2. Control Plugins
- `libgazebo_ros_control.so`: ROS control interfaces
- Custom plugins for specialized control algorithms

### 3. Visualization Plugins
- `libgazebo_ros_pcl.so`: Point cloud processing
- Custom plugins for specialized visualization

### Example Plugin Integration:
```xml
<model name="my_robot">
  <!-- Robot model definition -->
  
  <!-- Camera sensor -->
  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <update_rate>30.0</update_rate>
      <camera name="head">
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>800</width>
          <height>600</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <frame_name>camera_link</frame_name>
        <min_depth>0.1</min_depth>
        <max_depth>100</max_depth>
      </plugin>
    </sensor>
  </gazebo>
</model>
```

## Integration with ROS 2

### Setting up ROS 2 Communication:
```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Source workspace with robot description
source install/setup.bash

# Launch Gazebo with ROS 2 bridge
ros2 launch gazebo_ros empty_world.launch.py paused:=true use_sim_time:=true
```

### Topics and Services:
Gazebo exposes several ROS 2 interfaces:
- `/clock`: Simulation time
- `/gazebo/model_states`: All model poses
- `/gazebo/link_states`: All link poses
- `/gazebo/set_model_state`: Set model state
- `/gazebo/set_link_state`: Set link state

### Controlling Robots:
```python
import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Create publishers for joint control
        self.joint_cmd_pub = self.create_publisher(
            Float64, '/my_robot/joint1_position_controller/commands', 10)
        
        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        # Send control command
        msg = Float64()
        msg.data = 1.0  # Example position
        self.joint_cmd_pub.publish(msg)
```

## Common Issues and Troubleshooting

### 1. Performance Issues
- **High CPU usage**: Reduce physics update rate
- **Low frame rate**: Simplify collision meshes or reduce rendering quality
- **Instability**: Check time step and solver parameters

### 2. Model Errors
- **Falling through ground**: Check mass and collision properties
- **Jittering joints**: Tune joint limits and dynamics parameters
- **Floating objects**: Verify inertial properties and mass

### 3. ROS Integration Issues
- **No communication**: Check ROS_DOMAIN_ID and network configuration
- **Topic delays**: Optimize update rates and buffer sizes
- **Synchronization**: Ensure use_sim_time is configured properly

## Best Practices

### 1. World Design
- Start simple and gradually increase complexity
- Use efficient collision meshes
- Balance visual fidelity with performance

### 2. Model Creation
- Follow URDF/SDF best practices
- Include proper inertial properties
- Validate models in isolation before complex scenarios

### 3. Performance Optimization
- Use appropriate physics parameters
- Limit the number of complex objects
- Consider level-of-detail approaches

## Summary

This chapter introduced the fundamentals of Gazebo for robotics simulation. We covered installation, basic usage, world creation, and robot integration. Gazebo provides a powerful platform for testing and validating robotic systems before deployment on physical hardware.

Understanding the core components and architecture of Gazebo is essential for effective simulation-based development. Proper configuration of physics parameters, collision properties, and ROS integration enables realistic robot simulation for development and testing.

## Exercises

1. Create a simple Gazebo world with a ground plane and a few obstacles.
2. Spawn a robot model in Gazebo and verify it's properly positioned.
3. Add a camera sensor to your robot and verify it publishes images to ROS 2.
4. Configure basic joint controllers for your robot and test simple movements in simulation.