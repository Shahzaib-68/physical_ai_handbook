---
sidebar_position: 3
slug: /module-1/chapter-1-3
title: Chapter 1.3 - Bridging Python Agents to ROS Controllers (rclpy)
---

# Chapter 1.3: Bridging Python Agents to ROS Controllers (rclpy)

## Overview

As AI agents become more sophisticated, integrating them with robotic actuators and sensors is crucial. Python, being the dominant language in AI development, needs effective bridges to ROS-controlled robotic systems. The `rclpy` library provides the Python client API for ROS 2, enabling seamless integration between AI agents and robotic controllers.

## Learning Objectives

By the end of this chapter, students will be able to:
- Implement ROS 2 nodes using the `rclpy` library
- Bridge AI agent decisions to ROS 2 control commands
- Subscribe to sensor data and use it for AI decision-making
- Design proper message passing between Python AI components and ROS systems
- Handle timing constraints and synchronization between AI and control systems

## Introduction to rclpy

`rclpy` is the official Python client library for ROS 2. It provides Python bindings for the ROS 2 client library (rcl) and allows Python developers to create ROS 2 nodes, publishers, subscribers, and services.

### Core Components of rclpy:
- **Node**: The basic unit of computational organization
- **Publisher**: Interface for sending messages to topics
- **Subscriber**: Interface for receiving messages from topics
- **Service Client**: Interface for calling services
- **Service Server**: Interface for providing services
- **Action Client/Server**: Interface for long-running tasks with feedback

## Setting Up rclpy

Before implementing any ROS 2 node in Python, ensure proper setup:

1. Activate ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash  # Replace 'humble' with your ROS distro
```

2. Install dependencies:
```python
# In your package.xml
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```

3. Import necessary modules:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
```

## Implementing a Basic Bridge Node

Let's create a bridge that connects an AI agent's decisions to robot control commands:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np
import tensorflow as tf  # Example AI library

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')
        
        # Publisher for sending velocity commands to the robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber for sensor data
        self.sensor_subscriber = self.create_subscription(
            String,  # Using String for simplicity; use appropriate sensor_msgs
            '/sensor_data',
            self.sensor_callback,
            10
        )
        
        # Timer for AI decision loop
        self.timer = self.create_timer(0.1, self.ai_decision_loop)  # 10 Hz
        
        # Initialize AI model (dummy example)
        self.ai_model = self.initialize_model()
        
        # Store sensor data
        self.sensor_data = None
        self.ai_output = None
        
    def initialize_model(self):
        """Initialize AI model - replace with actual initialization"""
        # Example: Load pre-trained model
        # return tf.keras.models.load_model('path_to_model.h5')
        return "dummy_model"
    
    def sensor_callback(self, msg):
        """Handle incoming sensor data"""
        self.sensor_data = msg.data
        self.get_logger().info(f'Received sensor data: {self.sensor_data}')
    
    def ai_decision_loop(self):
        """Main AI decision-making loop"""
        if self.sensor_data is not None:
            # Process sensor data through AI model
            self.ai_output = self.process_with_ai(self.sensor_data)
            
            # Convert AI output to robot commands
            cmd_vel_msg = self.convert_to_robot_cmd(self.ai_output)
            
            # Publish command to robot
            self.cmd_vel_publisher.publish(cmd_vel_msg)
    
    def process_with_ai(self, sensor_data):
        """Process sensor data through AI model"""
        # This is where you'd normally call your AI model
        # For example: ai_output = self.ai_model.predict(sensor_data)
        
        # Dummy processing for example
        processed_data = np.random.rand(2)  # Linear and angular velocities
        return processed_data
    
    def convert_to_robot_cmd(self, ai_output):
        """Convert AI output to robot command message"""
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(ai_output[0])
        cmd_vel_msg.angular.z = float(ai_output[1])
        return cmd_vel_msg

def main(args=None):
    rclpy.init(args=args)
    ai_bridge_node = AIBridgeNode()
    
    try:
        rclpy.spin(ai_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating with Popular AI Libraries

### TensorFlow/Keras Integration:
```python
import tensorflow as tf

class TensorflowBridgeNode(Node):
    def __init__(self):
        super().__init__('tf_bridge_node')
        # Initialize TensorFlow model
        self.model = tf.keras.models.load_model('/path/to/model')
        
    def predict_action(self, observation):
        # Convert observation to tensor
        obs_tensor = tf.constant(observation, dtype=tf.float32)
        
        # Get prediction from model
        action = self.model.predict(tf.expand_dims(obs_tensor, axis=0))
        
        return action[0]
```

### PyTorch Integration:
```python
import torch
import torch.nn as nn

class PytorchBridgeNode(Node):
    def __init__(self):
        super().__init__('pt_bridge_node')
        # Initialize PyTorch model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = MyPytorchModel()
        self.model.load_state_dict(torch.load('/path/to/model.pth'))
        self.model.to(self.device)
        self.model.eval()  # Set to evaluation mode
        
    def predict_action(self, observation):
        # Convert to tensor and move to device
        obs_tensor = torch.tensor(observation, dtype=torch.float32).to(self.device)
        
        # Get prediction
        with torch.no_grad():
            action = self.model(obs_tensor)
            
        return action.cpu().numpy()
```

## Handling Multiple Sensor Modalities

Robots typically use multiple sensors (cameras, LiDAR, IMUs, joint encoders). Here's how to integrate different sensor types:

```python
from sensor_msgs.msg import Image, LaserScan, Imu
from cv_bridge import CvBridge
import cv2

class MultiModalBridge(Node):
    def __init__(self):
        super().__init__('multi_modal_bridge')
        
        # Initialize CV Bridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Subscribers for different sensor modalities
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        
        # Storage for sensor data
        self.latest_image = None
        self.latest_lidar = None
        self.latest_imu = None
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
        except Exception as e:
            self.get_logger().error(f'Could not convert image: {e}')
    
    def lidar_callback(self, msg):
        # Process LiDAR scan data
        ranges = np.array(msg.ranges)
        # Handle invalid range values
        ranges[np.isnan(ranges)] = 0.0
        ranges[np.isinf(ranges)] = msg.range_max
        self.latest_lidar = ranges
    
    def imu_callback(self, msg):
        # Extract orientation and acceleration data
        orientation = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
        
        angular_velocity = {
            'x': msg.angular_velocity.x,
            'y': msg.angular_velocity.y,
            'z': msg.angular_velocity.z
        }
        
        linear_acceleration = {
            'x': msg.linear_acceleration.x,
            'y': msg.linear_acceleration.y,
            'z': msg.linear_acceleration.z
        }
        
        self.latest_imu = {
            'orientation': orientation,
            'angular_velocity': angular_velocity,
            'linear_acceleration': linear_acceleration
        }
```

## Synchronization Issues

One of the most common challenges when bridging AI and robotics is handling timing differences:

### Problem: AI models often run at different frequencies than robot control systems
### Solution: Use buffering and interpolation techniques

```python
import collections
from scipy.interpolate import interp1d

class SynchronizedBridge(Node):
    def __init__(self):
        super().__init__('sync_bridge')
        
        # Buffer for sensor data with timestamps
        self.sensor_buffer = collections.deque(maxlen=100)
        
        # AI decision frequency (lower)
        self.ai_freq = 1.0  # Hz
        self.ai_timer = self.create_timer(1.0/self.ai_freq, self.ai_loop)
        
        # Control command frequency (higher)
        self.ctrl_freq = 50.0  # Hz
        self.ctrl_timer = self.create_timer(1.0/self.ctrl_freq, self.control_loop)
        
        # Current AI decision and target
        self.current_target = np.zeros(2)
        self.interpolated_target = np.zeros(2)
        self.target_start_time = self.get_clock().now()
        
    def sensor_callback(self, msg):
        # Timestamp sensor data
        timestamp = self.get_clock().now()
        self.sensor_buffer.append((timestamp, msg.data))
    
    def ai_loop(self):
        """AI decision-making at lower frequency"""
        if len(self.sensor_buffer) > 0:
            # Get most recent sensor data
            _, latest_sensor = self.sensor_buffer[-1]
            
            # Make new decision
            new_target = self.process_with_ai(latest_sensor)
            
            # Start interpolation to new target
            self.current_target = new_target
            self.target_start_time = self.get_clock().now()
    
    def control_loop(self):
        """Send control commands at higher frequency"""
        if self.get_clock().now().nanoseconds > self.target_start_time.nanoseconds:
            # Calculate interpolation factor (0 to 1)
            elapsed = (self.get_clock().now() - self.target_start_time).nanoseconds / 1_000_000_000.0
            alpha = min(elapsed * self.ai_freq, 1.0)  # Clamp to [0, 1]
            
            # Interpolate to target
            interpolated = self.interpolated_target * (1 - alpha) + self.current_target * alpha
            self.interpolated_target = interpolated
            
            # Publish control command
            cmd_msg = self.convert_to_robot_cmd(interpolated)
            self.cmd_publisher.publish(cmd_msg)
```

## Error Handling and Robustness

When bridging AI systems to physical robots, error handling is critical:

```python
class RobustBridgeNode(Node):
    def __init__(self):
        super().__init__('robust_bridge')
        
        # Watchdog timer to detect AI failures
        self.ai_watchdog = self.create_timer(2.0, self.check_ai_status)
        self.last_ai_update = self.get_clock().now()
        
        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def ai_decision_loop(self):
        try:
            # Update watchdog timer
            self.last_ai_update = self.get_clock().now()
            
            # Process AI decision
            if self.sensor_data is not None:
                self.ai_output = self.process_with_ai(self.sensor_data)
                
                # Validate AI output before sending to robot
                if self.validate_ai_output(self.ai_output):
                    cmd_vel_msg = self.convert_to_robot_cmd(self.ai_output)
                    self.cmd_vel_publisher.publish(cmd_vel_msg)
                else:
                    self.get_logger().warn('Invalid AI output detected, stopping robot')
                    self.emergency_stop()
                    
        except Exception as e:
            self.get_logger().error(f'AI decision loop error: {e}')
            self.emergency_stop()
    
    def validate_ai_output(self, output):
        """Validate AI output for safety"""
        # Check for NaN values
        if np.any(np.isnan(output)):
            return False
            
        # Check for extreme values
        if np.any(np.abs(output) > 10.0):  # Assuming max reasonable value is 10
            return False
            
        return True
    
    def emergency_stop(self):
        """Emergency stop function"""
        stop_msg = Twist()
        self.emergency_stop_pub.publish(stop_msg)
    
    def check_ai_status(self):
        """Check if AI is still responding"""
        time_since_update = self.get_clock().now() - self.last_ai_update
        if time_since_update.nanoseconds > 3_000_000_000:  # 3 seconds
            self.get_logger().warn('AI system not responding, stopping robot')
            self.emergency_stop()
```

## Best Practices

### 1. Proper Resource Management
Always properly cleanup resources, especially when using AI models with GPU memory:

```python
def destroy_node(self):
    # Clear any AI models to free memory
    self.ai_model = None
    super().destroy_node()
```

### 2. Configuration through Parameters
Allow runtime configuration of AI models and behavior:

```python
def __init__(self):
    super().__init__('configurable_ai_bridge')
    
    # Declare parameters
    self.declare_parameter('model_path', '/default/model/path')
    self.declare_parameter('control_frequency', 50)
    self.declare_parameter('ai_frequency', 1)
    
    # Get parameter values
    model_path = self.get_parameter('model_path').get_parameter_value().string_value
    ctrl_freq = self.get_parameter('control_frequency').get_parameter_value().integer_value
    ai_freq = self.get_parameter('ai_frequency').get_parameter_value().integer_value
```

### 3. Logging and Monitoring
Provide appropriate logging for debugging AI-robot integration issues:

```python
def ai_decision_loop(self):
    if self.sensor_data is not None:
        self.get_logger().debug(f'Sensor data shape: {np.shape(self.sensor_data)}')
        
        start_time = self.get_clock().now()
        self.ai_output = self.process_with_ai(self.sensor_data)
        ai_time = (self.get_clock().now() - start_time).nanoseconds / 1_000_000.0
        
        self.get_logger().info(f'AI processing took {ai_time:.2f} ms')
```

## Summary

This chapter demonstrated how to bridge AI agents implemented in Python with ROS-controlled robotic systems using `rclpy`. We covered:
- Basic node implementation with publishers and subscribers
- Integration with popular AI libraries (TensorFlow, PyTorch)
- Handling multiple sensor modalities
- Solving synchronization problems
- Ensuring safety through proper error handling

The techniques outlined enable AI researchers and roboticists to seamlessly connect sophisticated models with physical robotic platforms, forming the foundation for intelligent robotic systems.

## Exercises

1. Implement a bridge node that takes camera images, applies a pre-trained object detection model, and publishes detected object positions to a ROS topic.
2. Create a reinforcement learning agent that learns to control a simulated robot arm through ROS topics.
3. Develop a sensor fusion node that combines data from a camera, LiDAR, and IMU to improve robot perception.