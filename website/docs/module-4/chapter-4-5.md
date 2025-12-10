---
sidebar_position: 20
slug: /module-4/chapter-4-5
title: Chapter 4.5 - Capstone Project - Autonomous Humanoid
---

# Chapter 4.5: Capstone Project - Autonomous Humanoid

## Overview

This capstone project integrates all the concepts learned throughout the textbook to create an autonomous humanoid robot capable of understanding natural language commands, executing complex tasks, and operating safely in human environments. The project combines ROS 2, perception systems, language understanding, and humanoid control to create a complete autonomous system.

## Learning Objectives

By the end of this chapter, students will be able to:
- Integrate multiple robotics subsystems into a coherent autonomous system
- Implement end-to-end natural language processing for humanoid control
- Design and implement safety systems for autonomous operation
- Create complex task planning and execution pipelines
- Evaluate the performance of a complete humanoid robot system
- Troubleshoot and debug complex autonomous robot systems

## Project Overview

The autonomous humanoid robot will be capable of:
1. Understanding natural language commands through speech recognition
2. Processing commands using LLM-based cognitive planning
3. Navigating safely in human environments
4. Manipulating objects using humanoid arms
5. Operating safely with emergency stop capabilities
6. Providing feedback through speech and visual indicators

### System Architecture:

```
[User Commands] → [Voice Recognition] → [LLM Cognitive Planning] → [Task Execution] → [Robot Actions]
       ↓                  ↓                       ↓                      ↓              ↓
[Speech Input] ← [Text Processing] ← [Plan Validation] ← [Sensor Feedback] ← [Actuator Control]
```

## System Integration

### Main Autonomous Humanoid Node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState, Image, LaserScan
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import json
from typing import Dict, List, Optional, Any

# Import components from previous chapters
from .voice_interface import WhisperVoiceInterface
from .llm_planner import LLMBasedCognitivePlanner
from .perception_system import PerceptionSystem
from .navigation_system import NavigationSystem
from .manipulation_system import ManipulationSystem


class AutonomousHumanoid(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid')
        
        # Initialize subsystems
        self.voice_interface = WhisperVoiceInterface(self)
        self.llm_planner = LLMBasedCognitivePlanner(self)
        self.perception_system = PerceptionSystem(self)
        self.navigation_system = NavigationSystem(self)
        self.manipulation_system = ManipulationSystem(self)
        
        # Publishers for system control
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.feedback_pub = self.create_publisher(String, '/humanoid_feedback', 10)
        
        # Subscribers for system monitoring
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.emergency_stop_callback, 10)
        
        # System state
        self.system_active = True
        self.current_task = None
        self.robot_mode = "idle"  # idle, planning, executing, paused, error
        self.safety_override = False
        
        # Task queue for command execution
        self.task_queue = []
        self.task_queue_lock = threading.Lock()
        
        # Create timers
        self.status_timer = self.create_timer(1.0, self.publish_system_status)
        self.main_loop_timer = self.create_timer(0.1, self.main_control_loop)
        
        # Initialize safety systems
        self.safety_manager = SafetyManager(self)
        
        # Performance metrics
        self.metrics_collector = MetricsCollector()
        
        self.get_logger().info("Autonomous Humanoid System Initialized")
        self.publish_feedback("Autonomous humanoid system is ready", "system_ready")

    def emergency_stop_callback(self, msg):
        """Handle emergency stop commands"""
        if msg.data:
            self.get_logger().warn("Emergency stop activated!")
            self.safety_manager.activate_emergency_stop()
            self.robot_mode = "emergency"
            self.publish_feedback("EMERGENCY STOP ACTIVATED", "emergency")
        else:
            self.safety_manager.deactivate_emergency_stop()
            self.robot_mode = "idle"
            self.publish_feedback("System resumed", "resume")

    def main_control_loop(self):
        """Main control loop for the autonomous humanoid"""
        if not self.system_active or self.safety_override:
            return
        
        # Check safety status
        if not self.safety_manager.is_safe_to_operate():
            self.get_logger().warn("Safety check failed, pausing operations")
            self.robot_mode = "paused"
            return
        
        # Process task queue
        with self.task_queue_lock:
            if self.task_queue and self.robot_mode == "idle":
                # Start processing next task
                task = self.task_queue.pop(0)
                self.current_task = task
                self.robot_mode = "planning"
                
                # Plan and execute the task
                self.plan_and_execute_task(task)
        
        # Monitor current task execution
        if self.current_task and self.robot_mode == "executing":
            # Check if task completed or failed
            execution_status = self.monitor_task_execution()
            if execution_status == "completed":
                self.robot_mode = "idle"
                self.current_task = None
                self.publish_feedback("Task completed successfully", "success")
            elif execution_status == "failed":
                self.robot_mode = "error"
                self.publish_feedback("Task failed, requires assistance", "error")
                # TODO: Implement recovery or request human assistance

    def plan_and_execute_task(self, task_description: str):
        """Plan and execute a high-level task"""
        try:
            # Step 1: Use LLM to generate task plan
            self.get_logger().info(f"Planning task: {task_description}")
            plan = self.llm_planner.generate_plan_with_validation(task_description)
            
            if not plan:
                self.get_logger().error(f"Could not generate plan for: {task_description}")
                self.publish_feedback(f"Could not understand task: {task_description}", "error")
                self.robot_mode = "idle"
                return
            
            # Step 2: Validate plan with safety systems
            if not self.safety_manager.validate_plan(plan):
                self.get_logger().error("Plan failed safety validation")
                self.publish_feedback("Task plan failed safety validation", "error")
                self.robot_mode = "idle"
                return
            
            # Step 3: Execute the plan
            self.get_logger().info(f"Executing plan with {len(plan)} steps")
            self.robot_mode = "executing"
            
            execution_success = self.execute_plan(plan)
            
            if execution_success:
                self.get_logger().info("Plan executed successfully")
                self.publish_feedback("Task completed", "success")
            else:
                self.get_logger().error("Plan execution failed")
                self.publish_feedback("Task execution failed", "error")
            
            self.robot_mode = "idle"
            self.current_task = None
            
        except Exception as e:
            self.get_logger().error(f"Error planning and executing task: {e}")
            self.publish_feedback("System error during task execution", "error")
            self.robot_mode = "error"
            self.current_task = None

    def execute_plan(self, plan: List[Dict[str, Any]]) -> bool:
        """Execute a plan step by step"""
        for step_idx, step in enumerate(plan):
            self.get_logger().info(f"Executing step {step_idx + 1}/{len(plan)}: {step.get('action', 'unknown')}")
            
            # Check safety before each step
            if not self.safety_manager.is_safe_to_execute_step(step):
                self.get_logger().error(f"Step {step_idx} failed safety check")
                return False
            
            # Execute the step based on its action type
            success = self.execute_step(step)
            
            if not success:
                self.get_logger().error(f"Step {step_idx} failed: {step}")
                return False
            
            # Add metrics for this step
            self.metrics_collector.record_step_execution(step_idx, success)
        
        return True

    def execute_step(self, step: Dict[str, Any]) -> bool:
        """Execute a single plan step"""
        action = step.get('action', '').lower()
        
        if action == 'navigate':
            return self.navigation_system.execute_navigation(step)
        elif action == 'detect_object':
            return self.perception_system.execute_object_detection(step)
        elif action == 'grasp':
            return self.manipulation_system.execute_grasp(step)
        elif action == 'place':
            return self.manipulation_system.execute_place(step)
        elif action == 'speak':
            return self.speak(step.get('parameters', {}).get('text', ''))
        elif action == 'wait':
            wait_time = step.get('parameters', {}).get('time', 1.0)
            time.sleep(wait_time)
            return True
        else:
            self.get_logger().warn(f"Unknown action: {action}")
            return False

    def monitor_task_execution(self) -> str:
        """Monitor ongoing task execution"""
        # In a real implementation, this would monitor system states
        # and return the current status
        return "executing"  # Placeholder

    def add_task_to_queue(self, task_description: str):
        """Add a task to the execution queue"""
        with self.task_queue_lock:
            self.task_queue.append(task_description)
            self.get_logger().info(f"Added task to queue: {task_description}")
            self.publish_feedback(f"Task queued: {task_description[:50]}...", "queued")

    def publish_system_status(self):
        """Publish current system status"""
        status_msg = String()
        status_msg.data = json.dumps({
            "timestamp": self.get_clock().now().nanoseconds,
            "mode": self.robot_mode,
            "active": self.system_active,
            "task_queue_length": len(self.task_queue),
            "current_task": self.current_task,
            "safety_status": self.safety_manager.get_status()
        })
        
        self.system_status_pub.publish(status_msg)

    def speak(self, text: str) -> bool:
        """Make the robot speak"""
        try:
            # Publish to speech system
            speech_msg = String()
            speech_msg.data = text
            # self.speech_pub.publish(speech_msg)  # Would use actual speech topic
            
            self.get_logger().info(f"Speaking: {text}")
            return True
        except Exception as e:
            self.get_logger().error(f"Error in speech system: {e}")
            return False

    def publish_feedback(self, message: str, message_type: str = "info"):
        """Publish feedback to users"""
        feedback_msg = String()
        feedback_msg.data = json.dumps({
            "message": message,
            "type": message_type,
            "timestamp": self.get_clock().now().nanoseconds
        })
        
        self.feedback_pub.publish(feedback_msg)
        self.get_logger().info(f"Feedback: {message} [{message_type}]")


class SafetyManager:
    def __init__(self, node: Node):
        self.node = node
        self.emergency_active = False
        self.safety_limits = {
            "max_velocity": 0.5,  # m/s
            "max_joint_speed": 1.0,  # rad/s
            "min_obstacle_distance": 0.5,  # m
            "max_payload": 2.0,  # kg
        }
        
        # Subscribe to safety-relevant topics
        self.lidar_sub = self.node.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.last_scan = None
        self.last_joint_state = None
        
        self.get_logger().info("Safety Manager Initialized")

    def lidar_callback(self, msg):
        """Handle LiDAR data for obstacle detection"""
        self.last_scan = msg

    def joint_state_callback(self, msg):
        """Handle joint state data for safety checks"""
        self.last_joint_state = msg

    def get_logger(self):
        return self.node.get_logger()

    def activate_emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_active = True
        # TODO: Actually stop all robot motion
        self.get_logger().warn("EMERGENCY STOP ACTIVATED")

    def deactivate_emergency_stop(self):
        """Deactivate emergency stop"""
        self.emergency_active = False
        self.get_logger().info("Emergency stop deactivated")

    def is_safe_to_operate(self) -> bool:
        """Check if it's safe for the robot to operate"""
        if self.emergency_active:
            return False
        
        # Check for obstacles in path
        if self.last_scan and self.is_path_blocked():
            self.get_logger().warn("Path is blocked, not safe to proceed")
            return False
        
        # Check joint limits
        if self.last_joint_state and not self.are_joints_safe():
            self.get_logger().warn("Joint safety limits exceeded")
            return False
        
        return True

    def is_path_blocked(self) -> bool:
        """Check if robot's path is blocked by obstacles"""
        if not self.last_scan:
            return False  # No data, assume safe
        
        # Check for obstacles within minimum distance
        min_distance = self.safety_limits["min_obstacle_distance"]
        
        for i, range_val in enumerate(self.last_scan.ranges):
            if 0 < range_val < min_distance:
                # Obstacle detected
                angle = self.last_scan.angle_min + i * self.last_scan.angle_increment
                self.get_logger().debug(f"Obstacle detected at {range_val:.2f}m, angle {angle:.2f}")
                return True
        
        return False

    def are_joints_safe(self) -> bool:
        """Check if joint positions and velocities are within safe limits"""
        if not self.last_joint_state:
            return True  # No data, assume safe
        
        max_speed = self.safety_limits["max_joint_speed"]
        
        # Check joint velocities if available
        if self.last_joint_state.velocity:
            for vel in self.last_joint_state.velocity:
                if abs(vel) > max_speed:
                    return False
        
        # Additional joint safety checks would go here
        
        return True

    def validate_plan(self, plan: List[Dict[str, Any]]) -> bool:
        """Validate a plan for safety"""
        for step in plan:
            if not self.validate_step(step):
                self.get_logger().warn(f"Plan validation failed for step: {step}")
                return False
        return True

    def validate_step(self, step: Dict[str, Any]) -> bool:
        """Validate a single plan step"""
        action = step.get('action', '').lower()
        
        if action == 'navigate':
            return self.validate_navigation_step(step)
        elif action == 'grasp':
            return self.validate_grasp_step(step)
        elif action == 'manipulate':
            return self.validate_manipulation_step(step)
        else:
            # For other actions, just check general safety
            return self.is_safe_to_operate()
    
    def validate_navigation_step(self, step: Dict[str, Any]) -> bool:
        """Validate navigation step"""
        # Check if destination is safe
        target_pose = step.get('parameters', {}).get('pose')
        if not target_pose:
            return False
        
        # Additional safety checks for navigation would go here
        return self.is_safe_to_operate()

    def validate_grasp_step(self, step: Dict[str, Any]) -> bool:
        """Validate grasp step"""
        # Check if object is safe to grasp
        obj_weight = step.get('parameters', {}).get('object_weight', 0)
        if obj_weight > self.safety_limits["max_payload"]:
            self.get_logger().warn(f"Object too heavy: {obj_weight}kg > {self.safety_limits['max_payload']}kg")
            return False
        
        # Additional grasp safety checks would go here
        return True

    def validate_manipulation_step(self, step: Dict[str, Any]) -> bool:
        """Validate manipulation step"""
        # Additional manipulation safety checks would go here
        return True

    def is_safe_to_execute_step(self, step: Dict[str, Any]) -> bool:
        """Check if it's safe to execute a specific step"""
        if self.emergency_active:
            return False
        
        return self.validate_step(step)

    def get_status(self) -> Dict[str, Any]:
        """Get safety system status"""
        return {
            "emergency_active": self.emergency_active,
            "obstacle_detected": self.is_path_blocked() if self.last_scan else False,
            "joints_safe": self.are_joints_safe() if self.last_joint_state else True
        }


class MetricsCollector:
    def __init__(self):
        self.task_metrics = {
            "total_tasks": 0,
            "successful_tasks": 0,
            "failed_tasks": 0,
            "avg_task_time": 0.0,
            "avg_plan_steps": 0.0
        }
        
        self.step_metrics = {
            "total_steps": 0,
            "successful_steps": 0,
            "failed_steps": 0
        }
        
        self.task_times = []
        self.plan_lengths = []

    def record_task_completion(self, task_time: float, plan_length: int, success: bool):
        """Record metrics for a completed task"""
        self.task_metrics["total_tasks"] += 1
        
        if success:
            self.task_metrics["successful_tasks"] += 1
            self.task_times.append(task_time)
            self.plan_lengths.append(plan_length)
        else:
            self.task_metrics["failed_tasks"] += 1
        
        # Update averages
        if self.task_times:
            self.task_metrics["avg_task_time"] = sum(self.task_times) / len(self.task_times)
        if self.plan_lengths:
            self.task_metrics["avg_plan_steps"] = sum(self.plan_lengths) / len(self.plan_lengths)

    def record_step_execution(self, step_idx: int, success: bool):
        """Record metrics for a step execution"""
        self.step_metrics["total_steps"] += 1
        if success:
            self.step_metrics["successful_steps"] += 1
        else:
            self.step_metrics["failed_steps"] += 1

    def get_summary(self) -> Dict[str, Any]:
        """Get summary of metrics"""
        return {
            "task_metrics": self.task_metrics,
            "step_metrics": self.step_metrics,
            "task_success_rate": self.task_metrics["successful_tasks"] / self.task_metrics["total_tasks"] if self.task_metrics["total_tasks"] > 0 else 0,
            "step_success_rate": self.step_metrics["successful_steps"] / self.step_metrics["total_steps"] if self.step_metrics["total_steps"] > 0 else 0
        }


def main(args=None):
    rclpy.init(args=args)
    
    # Create the autonomous humanoid node
    humanoid = AutonomousHumanoid()
    
    # Create a multi-threaded executor for better performance
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(humanoid)
    
    try:
        humanoid.get_logger().info("Starting Autonomous Humanoid System...")
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        humanoid.get_logger().info("Shutting down Autonomous Humanoid System...")
        humanoid.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


# Additional support classes would be implemented in separate modules
# Here are the basic structures:

class VoiceInterface:
    def __init__(self, node):
        self.node = node
        # Initialize voice recognition system
        pass
    
    def start_listening(self):
        # Start voice recognition
        pass
    
    def stop_listening(self):
        # Stop voice recognition
        pass
    
    def process_speech(self, audio_data):
        # Process speech to text
        pass

class LLMPlanner:
    def __init__(self, node):
        self.node = node
        # Initialize LLM for planning
        pass
    
    def generate_plan(self, command):
        # Generate plan using LLM
        pass
    
    def validate_plan(self, plan):
        # Validate plan safety
        pass

class PerceptionSystem:
    def __init__(self, node):
        self.node = node
        # Initialize perception subsystem
        pass
    
    def execute_object_detection(self, step):
        # Execute object detection step
        pass
    
    def get_object_pose(self, object_name):
        # Get pose of an object
        pass

class NavigationSystem:
    def __init__(self, node):
        self.node = node
        # Initialize navigation subsystem
        pass
    
    def execute_navigation(self, step):
        # Execute navigation step
        pass
    
    def get_current_pose(self):
        # Get robot's current pose
        pass

class ManipulationSystem:
    def __init__(self, node):
        self.node = node
        # Initialize manipulation subsystem
        pass
    
    def execute_grasp(self, step):
        # Execute grasp step
        pass
    
    def execute_place(self, step):
        # Execute place step
        pass

if __name__ == '__main__':
    main()
```

## Voice Command Interface

### Enhanced Voice Command Processing:

```python
class VoiceCommandProcessor:
    def __init__(self, humanoid_node):
        self.node = humanoid_node
        self.command_queue = []
        self.is_listening = False
        
        # Define command vocabularies
        self.navigation_commands = {
            "go to", "navigate to", "move to", "go", "travel to", "walk to"
        }
        self.manipulation_commands = {
            "pick up", "grasp", "get", "fetch", "bring", "take", "place", "put down"
        }
        self.control_commands = {
            "stop", "pause", "resume", "wait", "help", "cancel"
        }
        
        # Command patterns for more complex commands
        self.patterns = [
            (r'bring me (the )?(.+?)$', self.handle_fetch_command),
            (r'go to (the )?(kitchen|bedroom|living room|office)$', self.handle_navigation_command),
            (r'pick up (the )?(.+?)$', self.handle_pickup_command),
            (r'put (the )?(.+?) (down|away|there)$', self.handle_place_command),
        ]

    def process_voice_command(self, text_command):
        """Process a voice command"""
        self.node.get_logger().info(f"Processing voice command: {text_command}")
        
        # Check for control commands first
        if self.is_control_command(text_command):
            return self.handle_control_command(text_command)
        
        # Try pattern matching for complex commands
        for pattern, handler in self.patterns:
            import re
            match = re.match(pattern, text_command.lower())
            if match:
                return handler(match.groups())
        
        # If no pattern matches, add to humanoid's task queue
        self.node.add_task_to_queue(text_command)
        return True

    def is_control_command(self, command):
        """Check if command is a control command"""
        lower_cmd = command.lower()
        return any(ctrl in lower_cmd for ctrl in ['stop', 'pause', 'resume', 'wait', 'help', 'cancel'])

    def handle_control_command(self, command):
        """Handle system control commands"""
        lower_cmd = command.lower()
        
        if 'stop' in lower_cmd or 'pause' in lower_cmd:
            # Send emergency stop
            stop_msg = Bool()
            stop_msg.data = True
            self.node.emergency_stop_pub.publish(stop_msg)
            return True
        elif 'resume' in lower_cmd:
            # Resume operations
            stop_msg = Bool()
            stop_msg.data = False
            self.node.emergency_stop_pub.publish(stop_msg)
            return True
        elif 'help' in lower_cmd:
            self.node.publish_feedback("Available commands: go to [location], bring me [object], pick up [object], stop, resume")
            return True
        else:
            return False

    def handle_fetch_command(self, groups):
        """Handle fetch commands like 'bring me X'"""
        obj_name = groups[1] if groups[1] else groups[0]
        task = f"bring me {obj_name}"
        self.node.add_task_to_queue(task)
        return True

    def handle_navigation_command(self, groups):
        """Handle navigation commands"""
        location = groups[1] if groups[1] else groups[0]
        task = f"go to {location}"
        self.node.add_task_to_queue(task)
        return True

    def handle_pickup_command(self, groups):
        """Handle pickup commands"""
        obj_name = groups[1] if groups[1] else groups[0]
        task = f"pick up {obj_name}"
        self.node.add_task_to_queue(task)
        return True

    def handle_place_command(self, groups):
        """Handle place commands"""
        obj_name = groups[1] if groups[1] else groups[0]
        task = f"place {obj_name}"
        self.node.add_task_to_queue(task)
        return True


class AdvancedAutonomousHumanoid(AutonomousHumanoid):
    def __init__(self):
        super().__init__()
        
        # Initialize voice command processor
        self.voice_processor = VoiceCommandProcessor(self)
        
        # Subscribe to voice commands
        self.voice_command_sub = self.create_subscription(
            String, '/voice_command', self.voice_command_callback, 10)
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor(self)
        
        self.get_logger().info("Advanced Autonomous Humanoid System Initialized")

    def voice_command_callback(self, msg):
        """Handle voice command from speech recognition"""
        command = msg.data
        success = self.voice_processor.process_voice_command(command)
        
        if success:
            self.get_logger().info(f"Voice command processed: {command}")
        else:
            self.get_logger().warn(f"Failed to process voice command: {command}")
            self.publish_feedback(f"Sorry, I didn't understand: {command}", "error")

    def main_control_loop(self):
        """Enhanced main control loop with performance monitoring"""
        # Call parent implementation
        super().main_control_loop()
        
        # Update performance metrics
        self.performance_monitor.update()
        
        # Check for system performance issues
        if self.performance_monitor.is_performance_degraded():
            self.get_logger().warn("System performance is degraded")
            # Could trigger performance optimization or alert user


class PerformanceMonitor:
    def __init__(self, node):
        self.node = node
        self.metrics = {
            "cpu_usage": [],
            "memory_usage": [],
            "loop_time": [],
            "task_queue_length": []
        }
        self.max_samples = 100  # Keep last 100 samples
        
        # Performance thresholds
        self.thresholds = {
            "cpu_usage": 80.0,  # Percentage
            "memory_usage": 85.0,  # Percentage
            "loop_time": 0.2,  # Seconds
            "queue_length": 10  # Tasks
        }

    def update(self):
        """Update performance metrics"""
        import psutil
        import os
        
        # Get CPU usage
        cpu_percent = psutil.cpu_percent()
        self.metrics["cpu_usage"].append(cpu_percent)
        if len(self.metrics["cpu_usage"]) > self.max_samples:
            self.metrics["cpu_usage"] = self.metrics["cpu_usage"][-self.max_samples:]
        
        # Get memory usage
        memory_percent = psutil.virtual_memory().percent
        self.metrics["memory_usage"].append(memory_percent)
        if len(self.metrics["memory_usage"]) > self.max_samples:
            self.metrics["memory_usage"] = self.metrics["memory_usage"][-self.max_samples:]
        
        # Get loop time (how long since last update)
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, 'last_update_time'):
            self.last_update_time = current_time
        else:
            loop_time = current_time - self.last_update_time
            self.metrics["loop_time"].append(loop_time)
            if len(self.metrics["loop_time"]) > self.max_samples:
                self.metrics["loop_time"] = self.metrics["loop_time"][-self.max_samples:]
            self.last_update_time = current_time
        
        # Get task queue length
        queue_length = len(self.node.task_queue)
        self.metrics["task_queue_length"].append(queue_length)
        if len(self.metrics["task_queue_length"]) > self.max_samples:
            self.metrics["task_queue_length"] = self.metrics["task_queue_length"][-self.max_samples:]

    def is_performance_degraded(self) -> bool:
        """Check if system performance is degraded"""
        # Check if any metric exceeds threshold
        if self.get_current_cpu_usage() > self.thresholds["cpu_usage"]:
            return True
        if self.get_current_memory_usage() > self.thresholds["memory_usage"]:
            return True
        if self.get_current_loop_time() > self.thresholds["loop_time"]:
            return True
        if self.get_current_queue_length() > self.thresholds["queue_length"]:
            return True
        
        return False

    def get_current_cpu_usage(self) -> float:
        """Get current CPU usage"""
        if self.metrics["cpu_usage"]:
            return self.metrics["cpu_usage"][-1]
        return 0.0

    def get_current_memory_usage(self) -> float:
        """Get current memory usage"""
        if self.metrics["memory_usage"]:
            return self.metrics["memory_usage"][-1]
        return 0.0

    def get_current_loop_time(self) -> float:
        """Get current loop time"""
        if self.metrics["loop_time"]:
            return self.metrics["loop_time"][-1]
        return 0.0

    def get_current_queue_length(self) -> int:
        """Get current task queue length"""
        if self.metrics["task_queue_length"]:
            return self.metrics["task_queue_length"][-1]
        return 0

    def get_average_values(self) -> Dict[str, float]:
        """Get average values for all metrics"""
        averages = {}
        for key, values in self.metrics.items():
            if values:
                averages[key] = sum(values) / len(values)
            else:
                averages[key] = 0.0
        return averages
```

## System Testing and Evaluation

### Comprehensive Testing Framework:

```python
class SystemTester:
    def __init__(self, humanoid_node):
        self.node = humanoid_node
        self.test_results = {}
        self.test_sequence = [
            self.test_basic_navigation,
            self.test_object_manipulation,
            self.test_voice_interface,
            self.test_safety_systems,
            self.test_concurrent_operations,
            self.test_long_term_operation
        ]

    def run_all_tests(self):
        """Run all system tests"""
        self.node.get_logger().info("Starting system tests...")
        
        for i, test_func in enumerate(self.test_sequence):
            test_name = test_func.__name__
            self.node.get_logger().info(f"Running test {i+1}/{len(self.test_sequence)}: {test_name}")
            
            try:
                result = test_func()
                self.test_results[test_name] = result
                status = "PASS" if result else "FAIL"
                self.node.get_logger().info(f"Test {test_name}: {status}")
            except Exception as e:
                self.node.get_logger().error(f"Test {test_name} failed with exception: {e}")
                self.test_results[test_name] = False

        self.generate_test_report()
        return self.test_results

    def test_basic_navigation(self) -> bool:
        """Test basic navigation capabilities"""
        try:
            # Plan a simple navigation task
            nav_task = "go to kitchen"
            
            # Add task to queue
            self.node.add_task_to_queue(nav_task)
            
            # Wait for completion or timeout
            start_time = time.time()
            while self.node.robot_mode != "idle" and (time.time() - start_time) < 30:
                time.sleep(0.1)
            
            # Check if task completed successfully
            return self.node.robot_mode == "idle" and not self.node.current_task
        except Exception:
            return False

    def test_object_manipulation(self) -> bool:
        """Test basic object manipulation"""
        try:
            # Create a simple manipulation task
            manipulation_task = "pick up the red block"
            
            # Add task to queue
            self.node.add_task_to_queue(manipulation_task)
            
            # Wait for completion or timeout
            start_time = time.time()
            while self.node.robot_mode != "idle" and (time.time() - start_time) < 60:
                time.sleep(0.1)
            
            # Check if task completed successfully
            return self.node.robot_mode == "idle" and not self.node.current_task
        except Exception:
            return False

    def test_voice_interface(self) -> bool:
        """Test voice command processing"""
        try:
            # Simulate voice command
            voice_cmd = String()
            voice_cmd.data = "go to living room"
            
            # Process command through voice interface
            result = self.node.voice_processor.process_voice_command(voice_cmd.data)
            
            # Check if command was successfully queued
            return result and len(self.node.task_queue) > 0
        except Exception:
            return False

    def test_safety_systems(self) -> bool:
        """Test safety system responses"""
        try:
            # Temporarily trigger safety check failure
            original_method = self.node.safety_manager.is_safe_to_operate
            self.node.safety_manager.is_safe_to_operate = lambda: False
            
            # Try to execute a task
            self.node.add_task_to_queue("go forward")
            
            # Wait to see if task gets blocked
            time.sleep(1.0)
            
            # Restore original method
            self.node.safety_manager.is_safe_to_operate = original_method
            
            # Task should not be executing if safety system works
            return self.node.robot_mode != "executing"
        except Exception:
            return False

    def test_concurrent_operations(self) -> bool:
        """Test handling multiple operations"""
        try:
            # Add multiple tasks to queue
            tasks = ["go to kitchen", "go to bedroom", "go to office"]
            for task in tasks:
                self.node.add_task_to_queue(task)
            
            # Check that all tasks were added
            return len(self.node.task_queue) == len(tasks)
        except Exception:
            return False

    def test_long_term_operation(self) -> bool:
        """Test system stability over time"""
        try:
            # Record initial metrics
            initial_metrics = self.node.metrics_collector.get_summary()
            
            # Let system run for a period
            start_time = time.time()
            while time.time() - start_time < 10:  # Run for 10 seconds
                # Add occasional tasks
                if (time.time() - start_time) % 3 < 0.1:  # Every 3 seconds
                    self.node.add_task_to_queue("wait 0.5 seconds")
                time.sleep(0.1)
            
            # Check if system remained stable
            final_metrics = self.node.metrics_collector.get_summary()
            
            # System should not have entered error mode
            return self.node.robot_mode != "error"
        except Exception:
            return False

    def generate_test_report(self):
        """Generate comprehensive test report"""
        report = "Autonomous Humanoid System Test Report\n"
        report += "=" * 50 + "\n"
        
        passed = sum(1 for result in self.test_results.values() if result)
        total = len(self.test_results)
        
        report += f"Tests Passed: {passed}/{total}\n"
        report += f"Success Rate: {passed/total*100:.1f}%\n\n"
        
        for test_name, result in self.test_results.items():
            status = "PASS" if result else "FAIL"
            report += f"{test_name}: {status}\n"
        
        report += f"\nFinal System Metrics:\n"
        metrics = self.node.metrics_collector.get_summary()
        for key, value in metrics.items():
            report += f"  {key}: {value}\n"
        
        # Log report
        self.node.get_logger().info(report)


class EvaluationFramework:
    def __init__(self, humanoid_node):
        self.node = humanoid_node
        self.evaluation_metrics = {
            "task_success_rate": 0.0,
            "response_time_avg": 0.0,
            "safety_incidents": 0,
            "user_satisfaction": 0.0,
            "system_availability": 0.0
        }
        
        self.task_log = []
        self.start_time = time.time()

    def log_task_completion(self, task_description, success, response_time):
        """Log completion of a task for evaluation"""
        task_record = {
            "task": task_description,
            "success": success,
            "response_time": response_time,
            "timestamp": time.time()
        }
        
        self.task_log.append(task_record)

    def calculate_metrics(self):
        """Calculate evaluation metrics"""
        if not self.task_log:
            return self.evaluation_metrics
        
        # Calculate task success rate
        successful_tasks = [t for t in self.task_log if t["success"]]
        self.evaluation_metrics["task_success_rate"] = len(successful_tasks) / len(self.task_log)
        
        # Calculate average response time
        response_times = [t["response_time"] for t in self.task_log]
        if response_times:
            self.evaluation_metrics["response_time_avg"] = sum(response_times) / len(response_times)
        
        # Calculate system availability
        uptime = time.time() - self.start_time
        # Simplified availability calculation (in real system, track downtime)
        self.evaluation_metrics["system_availability"] = 0.95  # Placeholder
        
        return self.evaluation_metrics

    def generate_evaluation_report(self):
        """Generate comprehensive evaluation report"""
        metrics = self.calculate_metrics()
        
        report = "Autonomous Humanoid Evaluation Report\n"
        report += "=" * 50 + "\n"
        
        for metric, value in metrics.items():
            report += f"{metric}: {value}\n"
        
        report += f"\nTotal Tasks Processed: {len(self.task_log)}\n"
        report += f"Successful Tasks: {len([t for t in self.task_log if t['success']])}\n"
        report += f"Failed Tasks: {len([t for t in self.task_log if not t['success']])}\n"
        
        return report
```

## Implementation and Deployment

### System Deployment Configuration:

```python
# deployment_config.py
HUMANOID_CONFIG = {
    "robot": {
        "type": "humanoid",
        "model": "custom_humanoid_v1",
        "degrees_of_freedom": 24,
        "sensors": ["lidar", "camera", "imu", "joint_encoders"],
        "actuators": ["legs", "arms", "head"],
        "max_speed": 0.5,
        "max_payload": 2.0
    },
    "navigation": {
        "planner": "nav2",
        "local_planner": "dwb",
        "global_planner": "navfn",
        "costmap_resolution": 0.05,
        "update_frequency": 5.0
    },
    "manipulation": {
        "arm_dof": 7,
        "gripper_type": "parallel_jaw",
        "max_force": 50.0,
        "control_frequency": 100.0
    },
    "perception": {
        "camera_resolution": [640, 480],
        "detection_distance": 3.0,
        "object_classes": ["cup", "bottle", "book", "box", "person"],
        "tracking_enabled": True
    },
    "voice": {
        "model_size": "base",  # tiny, base, small, medium, large
        "language": "en",
        "beam_size": 5,
        "temperature": 0.0
    },
    "llm": {
        "provider": "openai",
        "model": "gpt-4",
        "api_key": "YOUR_API_KEY_HERE",  # In production, use secure storage
        "max_tokens": 1000,
        "temperature": 0.3
    },
    "safety": {
        "emergency_stop_timeout": 5.0,
        "max_velocity": 0.5,
        "min_obstacle_distance": 0.5,
        "collision_threshold": 10.0,  # N-m
        "monitoring_frequency": 10.0
    }
}

# launch_nodes.py
def launch_autonomous_humanoid_system():
    """Launch all required nodes for the autonomous humanoid system"""
    import subprocess
    import time
    
    nodes_to_launch = [
        # Perception nodes
        "ros2 run your_perception_package object_detector",
        "ros2 run your_perception_package pose_estimator", 
        
        # Navigation nodes
        "ros2 launch nav2_bringup navigation_launch.py",
        
        # Manipulation nodes
        "ros2 run your_manipulation_package arm_controller",
        "ros2 run your_manipulation_package gripper_controller",
        
        # Voice interface nodes
        "ros2 run your_voice_package speech_recognizer",
        
        # Main autonomous system
        f"ros2 run your_humanoid_package autonomous_humanoid --ros-args --params-file {HUMANOID_CONFIG}"
    ]
    
    processes = []
    for node_cmd in nodes_to_launch:
        process = subprocess.Popen(node_cmd.split())
        processes.append(process)
        time.sleep(2)  # Wait between launching nodes
    
    # Wait for processes to finish
    for process in processes:
        process.wait()
```

## Troubleshooting Guide

### Common Issues and Solutions:

```python
class TroubleshootingGuide:
    @staticmethod
    def get_troubleshooting_info(issue_type: str) -> str:
        """Get troubleshooting information for specific issue types"""
        troubleshooting_info = {
            "navigation_failure": """
            Navigation Failure Troubleshooting:
            1. Check if costmaps are updating: ros2 topic echo /local_costmap/costmap
            2. Verify robot localization: ros2 topic echo /amcl_pose
            3. Check for obstacle detection: ros2 topic echo /scan
            4. Validate map quality and resolution
            5. Confirm robot's physical state (batteries, drive systems)
            """,
            
            "manipulation_failure": """
            Manipulation Failure Troubleshooting:
            1. Verify object detection: ros2 topic echo /detected_objects
            2. Check arm joint states: ros2 topic echo /joint_states
            3. Confirm gripper status: ros2 topic echo /gripper_status
            4. Validate grasp planning with RViz visualization
            5. Check payload limits and object properties
            """,
            
            "voice_recognition": """
            Voice Recognition Issues:
            1. Check microphone connection: arecord -l
            2. Verify audio input: ros2 topic echo /audio_input
            3. Test Whisper model directly: python -c "import whisper; whisper.load_model('base')"
            4. Check ambient noise levels
            5. Validate language settings match spoken commands
            """,
            
            "llm_integration": """
            LLM Integration Issues:
            1. Verify API key and network connectivity
            2. Check prompt formatting and token limits
            3. Validate response parsing and error handling
            4. Monitor rate limits and quota usage
            5. Test LLM separately from robot system
            """,
            
            "safety_system": """
            Safety System Activation:
            1. Check emergency stop button state
            2. Verify sensor data (LiDAR, cameras) is streaming
            3. Confirm joint limits and velocities are within bounds
            4. Inspect environment for potential hazards
            5. Review safety configuration parameters
            """
        }
        
        return troubleshooting_info.get(issue_type, "Unknown issue type. Please consult the full debugging guide.")
```

## Summary and Best Practices

### System Integration Best Practices:

1. **Modular Design**: Keep subsystems as independent as possible with well-defined interfaces
2. **Safety First**: Implement safety checks at every level of the system
3. **Performance Monitoring**: Continuously monitor system performance and resource usage
4. **Error Handling**: Implement comprehensive error handling and recovery mechanisms
5. **Testing**: Develop thorough testing procedures for each subsystem and integration
6. **Maintenance**: Plan for system updates, debugging, and long-term maintenance

## Summary

This capstone project brought together all the concepts learned throughout the textbook to create a complete autonomous humanoid robot system. The project involved:

1. Integrating voice recognition with natural language processing
2. Implementing LLM-based cognitive planning
3. Developing perception and navigation systems
4. Creating manipulation capabilities
5. Implementing comprehensive safety systems
6. Designing performance monitoring and evaluation frameworks

The autonomous humanoid system represents a sophisticated integration of multiple AI and robotics technologies, demonstrating how modern tools like LLMs, advanced perception, and ROS 2 can work together to create capable autonomous robots.

Key achievements of the project include:
- Natural language interface for robot control
- Cognitive planning capabilities using LLMs
- Safe autonomous navigation and manipulation
- Comprehensive monitoring and safety systems
- Evaluation framework for system performance

This project serves as a foundation for further development in advanced robotics, AI integration, and human-robot interaction.

## Exercises

1. Implement the complete autonomous humanoid system with the provided code structure.
2. Add additional safety checks and validation mechanisms to the system.
3. Integrate an actual LLM service (like OpenAI API) with your robot system.
4. Develop a user interface for monitoring and controlling the autonomous humanoid.
5. Conduct extensive testing of the system in simulation and with real hardware.
6. Evaluate the system's performance with various human operators and tasks.
7. Extend the system to handle multi-person interactions and complex social scenarios.