---
sidebar_position: 4
slug: /module-1/chapter-1-4
title: Chapter 1.4 - URDF for Humanoid Robots
---

# Chapter 1.4: URDF for Humanoid Robots

## Overview

The Unified Robot Description Format (URDF) is an XML-based format used to describe robotic systems in ROS. For humanoid robots with their complex kinematic structures and multiple degrees of freedom, URDF is essential for modeling everything from joint configurations to physical properties. This chapter covers advanced URDF concepts specifically for humanoid robot modeling.

## Learning Objectives

By the end of this chapter, students will be able to:
- Define complex humanoid robot structures using URDF
- Model kinematic chains with multiple limbs
- Apply proper physical properties (mass, inertia, friction) to humanoid links
- Implement transmission mechanisms for actuator control
- Use Xacro macros to simplify complex humanoid descriptions
- Visualize and validate humanoid robot models in simulation

## Introduction to URDF for Humanoid Robots

Humanoid robots present unique challenges in URDF modeling compared to simpler robots:

- **Multiple limbs**: Each with multiple joints and links
- **Complex kinematics**: Interconnected limbs with closed kinematic chains
- **Balance requirements**: Precise center of mass for stable locomotion
- **Actuator complexity**: Various joint types and transmission systems
- **Collision detection**: Multiple potential collision pairs

## Basic URDF Structure for Humanoid Robots

A humanoid URDF follows the same basic structure as any robot, but with more complex link and joint arrangements:

```xml
<?xml version="1.0" ?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1.0" />
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://humanoid_description/meshes/base_link.stl" />
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <mesh filename="package://humanoid_description/meshes/base_link.stl" />
      </geometry>
    </collision>
  </link>

  <!-- Joints connect links -->
  <joint name="waist_joint" type="revolute">
    <parent link="base_link" />
    <child link="torso" />
    <origin xyz="0 0 0.5" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  </joint>

  <link name="torso">
    <inertial>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <mass value="5.0" />
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />
    </inertial>
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.15" length="0.6" />
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.15" length="0.6" />
      </geometry>
    </collision>
  </link>
</robot>
```

## Modeling Humanoid Limbs

Humanoid robots typically have:
- Head with neck joints
- Torso with waist joints
- Two arms with shoulder, elbow, and wrist joints
- Two legs with hip, knee, and ankle joints

Here's an example of how to model an arm:

```xml
<!-- Left Arm -->
<joint name="left_shoulder_yaw_joint" type="revolute">
  <parent link="torso" />
  <child link="left_upper_arm" />
  <origin xyz="0.1 0.2 0.4" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
  <dynamics damping="0.1" friction="0.0" />
</joint>

<link name="left_upper_arm">
  <inertial>
    <origin xyz="0 0 0.15" rpy="0 0 0" />
    <mass value="1.0" />
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005" />
  </inertial>
  <visual>
    <origin xyz="0 0 0.15" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.05" length="0.3" />
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.15" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.05" length="0.3" />
    </geometry>
  </collision>
</link>

<joint name="left_elbow_joint" type="revolute">
  <parent link="left_upper_arm" />
  <child link="left_lower_arm" />
  <origin xyz="0 0 0.3" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="0" upper="3.14" effort="100" velocity="1" />
</joint>

<link name="left_lower_arm">
  <inertial>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <mass value="0.5" />
    <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
  </inertial>
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.04" length="0.2" />
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.04" length="0.2" />
    </geometry>
  </collision>
</link>
```

## Proper Inertial Properties

Accurate inertial properties are critical for humanoid robot simulation:

### Mass Distribution:
- Head: ~5-10% of total body weight
- Torso: ~50% of total body weight
- Arms: ~5-10% each
- Legs: ~15-20% each

### Calculating Moments of Inertia:
For common shapes:
- Cylinder: `ixx = iyy = (1/12)*m*(3*r² + h²)`, `izz = (1/2)*m*r²`
- Sphere: `ixx = iyy = izz = (2/5)*m*r²`
- Box: `ixx = (1/12)*m*(h² + d²)`, `iyy = (1/12)*m*(w² + d²)`, `izz = (1/12)*m*(w² + h²)`

### Example with accurate inertial properties:
```xml
<link name="left_upper_leg">
  <inertial>
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <mass value="3.0" />
    <!-- Approximating as a cylinder -->
    <inertia ixx="0.03" ixy="0.0" ixz="0.0" iyy="0.03" iyz="0.0" izz="0.005" />
  </inertial>
  <visual>
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.07" length="0.4" />
    </geometry>
    <material name="yellow">
      <color rgba="1 1 0 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 -0.2" rpy="0 0 0" />
    <geometry>
      <cylinder radius="0.07" length="0.4" />
    </geometry>
  </collision>
</link>
```

## Joint Types for Humanoid Robots

Humanoid robots require various joint types for realistic movement:

### Revolute Joint
```xml
<joint name="knee_joint" type="revolute">
  <parent link="upper_leg" />
  <child link="lower_leg" />
  <origin xyz="0 0 -0.4" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="0" upper="2.5" effort="200" velocity="1.5" />
</joint>
```

### Continuous Joint (for unlimited rotation)
```xml
<joint name="neck_yaw_joint" type="continuous">
  <parent link="torso" />
  <child link="head" />
  <origin xyz="0 0 0.6" rpy="0 0 0" />
  <axis xyz="0 0 1" />
</joint>
```

### Fixed Joint (for non-moving connections)
```xml
<joint name="head_camera_joint" type="fixed">
  <parent link="head" />
  <child link="head_camera_frame" />
  <origin xyz="0.05 0 0.05" rpy="0 0 0" />
</joint>
```

## Transmissions for Actuator Control

To control joints with actuators, use transmissions:

```xml
<transmission name="left_knee_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_knee_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_knee_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Using Xacro for Complex Humanoid Models

Xacro (XML Macros) allows parameterization and reusability:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.6" />
  <xacro:property name="torso_radius" value="0.15" />
  <xacro:property name="arm_length" value="0.4" />

  <!-- Macro for creating an arm -->
  <xacro:macro name="arm" params="side reflect">
    <joint name="${side}_shoulder_yaw_joint" type="revolute">
      <parent link="torso" />
      <child link="${side}_upper_arm" />
      <origin xyz="${reflect * 0.1} 0.2 0.4" rpy="0 0 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1.57" upper="1.57" effort="100" velocity="1" />
    </joint>

    <link name="${side}_upper_arm">
      <inertial>
        <origin xyz="0 0 0.15" rpy="0 0 0" />
        <mass value="1.0" />
        <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005" />
      </inertial>
      <visual>
        <origin xyz="0 0 0.15" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.05" length="0.3" />
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1" />
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0.15" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.05" length="0.3" />
        </geometry>
      </collision>
    </link>

    <joint name="${side}_elbow_joint" type="revolute">
      <parent link="${side}_upper_arm" />
      <child link="${side}_lower_arm" />
      <origin xyz="0 0 0.3" rpy="0 0 0" />
      <axis xyz="0 1 0" />
      <limit lower="0" upper="3.14" effort="100" velocity="1" />
    </joint>

    <link name="${side}_lower_arm">
      <inertial>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <mass value="0.5" />
        <inertia ixx="0.002" ixy="0.0" ixz="0.0" iyy="0.002" iyz="0.0" izz="0.002" />
      </inertial>
      <visual>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.04" length="0.2" />
        </geometry>
        <material name="gray">
          <color rgba="0.5 0.5 0.5 1" />
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 0.1" rpy="0 0 0" />
        <geometry>
          <cylinder radius="0.04" length="0.2" />
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Use the macro to create arms -->
  <xacro:arm side="left" reflect="1" />
  <xacro:arm side="right" reflect="-1" />
</robot>
```

## Gazebo Integration

For simulation in Gazebo, add Gazebo-specific tags:

```xml
<!-- Gazebo materials -->
<gazebo reference="left_upper_arm">
  <material>Gazebo/Gray</material>
</gazebo>

<!-- Gazebo plugins for joint control -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
  </plugin>
</gazebo>

<!-- Gazebo sensors -->
<gazebo reference="head_camera_frame">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head_camera">
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
      <alwaysOn>true</alwaysOn>
      <updateRate>0.0</updateRate>
      <cameraName>head_camera</cameraName>
      <imageTopicName>image_raw</imageTopicName>
      <cameraInfoTopicName>camera_info</cameraInfoTopicName>
      <frameName>head_camera_frame</frameName>
    </plugin>
  </sensor>
</gazebo>
```

## Validation and Visualization

Before simulating, validate your URDF:

1. **Check for errors**:
```bash
check_urdf /path/to/robot.urdf
```

2. **Visualize in Rviz**:
```bash
ros2 launch urdf_tutorial display.launch.py model:=/path/to/robot.urdf
```

3. **Check kinematics**:
```bash
ros2 run tf2_tools view_frames
```

## Common Issues and Solutions

### 1. Massless Links
Every link must have a mass > 0 for physics simulation:
```xml
<!-- Good -->
<inertial>
  <mass value="0.1" />
  <!-- ... -->
</inertial>

<!-- Bad - causes physics errors -->
<inertial>
  <mass value="0" />
  <!-- ... -->
</inertial>
```

### 2. Incorrect Inertial Values
- Use realistic values based on the physical object
- Ensure the inertia matrix is positive definite
- For complex shapes, use CAD software to calculate accurate values

### 3. Kinematic Loops
Humanoid robots often have closed kinematic chains. Consider using `fixed_joint_inertia` tags or simplifying models for kinematic solvers.

## Summary

This chapter covered the essentials of creating URDF models for humanoid robots:
- Basic structure with links and joints
- Modeling complex multi-limb structures
- Proper inertial properties for stable simulation
- Using Xacro for complex, reusable models
- Integration with simulation and visualization tools

Accurate URDF modeling is crucial for successful humanoid robot simulation, control, and planning. The complexity of these models requires careful attention to physical properties and kinematic structure.

## Exercises

1. Create a URDF model of a simple humanoid robot with a head, torso, two arms, and two legs using Xacro macros.
2. Add proper inertial properties to the model based on typical human body segment parameters.
3. Include transmission definitions for all joints and simulate the robot in Gazebo.
4. Add sensors (camera, IMU) to the head of the robot and verify they are properly integrated.