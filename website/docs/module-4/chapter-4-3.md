---
sidebar_position: 18
slug: /module-4/chapter-4-3
title: Chapter 4.3 - Cognitive Planning with LLMs
---

# Chapter 4.3: Cognitive Planning with LLMs

## Overview

Cognitive planning with Large Language Models (LLMs) represents a significant advancement in robotic autonomy, enabling robots to reason about complex, multi-step tasks using high-level language descriptions. This chapter explores how LLMs can serve as cognitive planners that decompose high-level commands into executable robotic behaviors, incorporating world knowledge and reasoning capabilities into robotic planning processes.

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the role of LLMs in cognitive planning for robotics
- Design LLM-based planning architectures for robotic systems
- Implement task decomposition using LLMs
- Integrate LLM planning with traditional robotic planning systems
- Evaluate the effectiveness of LLM-based cognitive planning
- Address challenges in LLM planning such as hallucinations and grounding

## Introduction to Cognitive Planning

Cognitive planning involves high-level reasoning about tasks, goals, and the environment to produce a sequence of actions that accomplish objectives. Traditional robotic planning focuses on low-level motion and manipulation, while cognitive planning operates at a higher semantic level, working with concepts like "bring me a cup" rather than "move to pose (1.0, 2.0, 0.0)".

### Traditional vs. LLM-Based Planning:
- **Traditional**: Low-level, geometric, reactive
- **LLM-Based**: High-level, semantic, reasoning-based

### Benefits of LLM Cognitive Planning:
- Natural language task specification
- Access to world knowledge
- Ability to handle ambiguous or high-level goals
- Adaptability to novel situations
- Integration of common-sense reasoning

## LLM Cognitive Planning Architecture

### Architecture Components:

```
[High-Level Command] → [LLM Planner] → [Task Decomposition] → [Action Sequence] → [Robotic Execution]
         ↓                   ↓                  ↓                    ↓
[Robot State] ← [Context Update] ← [World Knowledge] ← [Execution Feedback]
```

### Core Components:
1. **Command Interpreter**: Translates natural language to planning tasks
2. **Knowledge Base**: Integrates world knowledge with robot knowledge
3. **Decomposition Engine**: Breaks high-level tasks into subtasks
4. **Plan Validator**: Ensures plan feasibility and safety
5. **Execution Monitor**: Tracks plan execution and adapts as needed

## Implementing LLM-Based Task Decomposition

### Basic Cognitive Planning Node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import openai
import json
import re
from typing import List, Dict, Any, Optional


class LLMBasedCognitivePlanner(Node):
    def __init__(self):
        super().__init__('llm_cognitive_planner')
        
        # Initialize LLM client
        # openai.api_key = "your-api-key-here"
        
        # Publishers and subscribers
        self.high_level_command_sub = self.create_subscription(
            String, '/high_level_command', self.high_level_command_callback, 10)
        
        self.task_sequence_pub = self.create_publisher(
            String, '/task_sequence', 10)
        
        self.robot_feedback_sub = self.create_subscription(
            String, '/robot_feedback', self.robot_feedback_callback, 10)
        
        # Internal state
        self.planning_context = {
            "robot_capabilities": self.get_robot_capabilities(),
            "environment_map": {},  # Will be populated from SLAM, object detection
            "object_locations": {},  # Will be populated from perception
            "robot_state": {}  # Will be populated from robot state
        }
        
        # Store current plan
        self.current_plan = []
        self.current_step_index = 0
        
        self.get_logger().info("LLM Cognitive Planner Initialized")

    def get_robot_capabilities(self) -> Dict[str, Any]:
        """Define robot capabilities for LLM context"""
        return {
            "navigation": True,
            "manipulation": True,
            "grasping": True,
            "object_detection": True,
            "speech": True,
            "arm_dof": 7,
            "max_reach": 1.5,  # meters
            "payload": 2.0     # kg
        }

    def high_level_command_callback(self, msg):
        """Handle high-level commands using LLM planning"""
        command = msg.data
        self.get_logger().info(f"Received high-level command: {command}")
        
        # Create plan using LLM
        plan = self.generate_plan_with_llm(command)
        
        if plan:
            self.current_plan = plan
            self.current_step_index = 0
            
            # Publish plan for execution
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.task_sequence_pub.publish(plan_msg)
            
            self.get_logger().info(f"Generated plan with {len(plan)} steps")
        else:
            self.get_logger().error("Failed to generate plan for command")

    def generate_plan_with_llm(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Generate a task plan using LLM"""
        try:
            # Create context-aware prompt for planning
            context = self.planning_context
            prompt = f"""
            You are an autonomous robot planning agent with the following capabilities:
            {json.dumps(context["robot_capabilities"], indent=2)}
            
            Current environment information (if available):
            - Object locations: {json.dumps(context["object_locations"], indent=2)}
            - Environment map: {json.dumps(context["environment_map"], indent=2)}
            
            Create a detailed plan to execute this command: "{command}"
            
            Return the plan as a JSON array of tasks, where each task has:
            {{
                "step_id": integer,
                "action": string (e.g., "navigate", "detect_object", "grasp", "transport", "place"),
                "description": string,
                "parameters": object,
                "preconditions": array of strings,
                "expected_outcomes": array of strings,
                "safety_considerations": array of strings
            }}
            
            Example plan for "Bring me a red cup from the kitchen":
            [
                {{
                    "step_id": 1,
                    "action": "navigate",
                    "description": "Navigate to kitchen area",
                    "parameters": {{"target_location": "kitchen"}},
                    "preconditions": ["robot_is_idle"],
                    "expected_outcomes": ["robot_at_kitchen"],
                    "safety_considerations": ["avoid_obstacles", "check_path_clear"]
                }},
                {{
                    "step_id": 2,
                    "action": "detect_object",
                    "description": "Detect red cup in kitchen",
                    "parameters": {{"object_type": "cup", "color": "red"}},
                    "preconditions": ["robot_at_kitchen"],
                    "expected_outcomes": ["location_of_red_cup", "grasp_pose_computed"],
                    "safety_considerations": ["check_object_stability", "avoid_damage"]
                }},
                {{
                    "step_id": 3,
                    "action": "grasp",
                    "description": "Grasp the red cup",
                    "parameters": {{"object_pose": "..."}},
                    "preconditions": ["object_detected", "robot_at_object_location"],
                    "expected_outcomes": ["object_grasped", "grasp_stable"],
                    "safety_considerations": ["check_grasp_force", "avoid_slip"]
                }},
                {{
                    "step_id": 4,
                    "action": "navigate",
                    "description": "Navigate to user location",
                    "parameters": {{"target_location": "user"}},
                    "preconditions": ["object_grasped"],
                    "expected_outcomes": ["robot_with_object_at_user"],
                    "safety_considerations": ["maintain_balance", "protect_object"]
                }},
                {{
                    "step_id": 5,
                    "action": "place",
                    "description": "Place cup near user",
                    "parameters": {{"placement_pose": "..."}},
                    "preconditions": ["robot_with_object_at_user"],
                    "expected_outcomes": ["object_placed", "robot_free"],
                    "safety_considerations": ["place_safely", "avoid_collision"]
                }}
            ]
            
            Ensure the plan is executable with the robot's capabilities and consider potential failures.
            """
            
            # In practice, use your preferred LLM API
            # response = openai.ChatCompletion.create(
            #     model="gpt-4",
            #     messages=[{"role": "user", "content": prompt}],
            #     temperature=0.3,
            #     max_tokens=2000
            # )
            # 
            # # Parse response
            # content = response.choices[0].message.content
            # # Extract JSON from the response
            # json_match = re.search(r'\[.*\]', content, re.DOTALL)
            # if json_match:
            #     plan = json.loads(json_match.group(0))
            #     return plan
            # else:
            #     self.get_logger().error(f"Could not extract JSON from LLM response: {content}")
            #     return None
            
            # For demonstration, return a mock plan
            return self.mock_plan_generation(command)
            
        except Exception as e:
            self.get_logger().error(f"Error generating plan with LLM: {e}")
            return None

    def mock_plan_generation(self, command: str) -> List[Dict[str, Any]]:
        """Generate mock plan for demonstration purposes"""
        # This would be replaced with actual LLM call in real implementation
        if "bring me" in command.lower() or "fetch" in command.lower():
            return [
                {
                    "step_id": 1,
                    "action": "navigate",
                    "description": "Navigate to object location",
                    "parameters": {"target_location": "kitchen"},
                    "preconditions": ["robot_is_idle"],
                    "expected_outcomes": ["robot_at_kitchen"],
                    "safety_considerations": ["avoid_obstacles"]
                },
                {
                    "step_id": 2,
                    "action": "detect_object",
                    "description": "Detect specified object",
                    "parameters": {"object_type": "cup"},
                    "preconditions": ["robot_at_kitchen"],
                    "expected_outcomes": ["object_location_known"],
                    "safety_considerations": ["check_object_safety"]
                },
                {
                    "step_id": 3,
                    "action": "grasp",
                    "description": "Grasp the object",
                    "parameters": {"grasp_pose": [0.5, 0, 1.0]},
                    "preconditions": ["object_detected"],
                    "expected_outcomes": ["object_grasped"],
                    "safety_considerations": ["check_grasp_stability"]
                },
                {
                    "step_id": 4,
                    "action": "navigate",
                    "description": "Navigate to user",
                    "parameters": {"target_location": "user"},
                    "preconditions": ["object_grasped"],
                    "expected_outcomes": ["robot_at_user"],
                    "safety_considerations": ["maintain_balance"]
                },
                {
                    "step_id": 5,
                    "action": "place",
                    "description": "Place object near user",
                    "parameters": {"placement_pose": [0, 0, 0.8]},
                    "preconditions": ["robot_at_user"],
                    "expected_outcomes": ["object_placed"],
                    "safety_considerations": ["place_safely"]
                }
            ]
        else:
            # Return a simple navigation plan for other commands
            return [
                {
                    "step_id": 1,
                    "action": "navigate",
                    "description": f"Navigate as per command: {command}",
                    "parameters": {"command": command},
                    "preconditions": ["robot_is_idle"],
                    "expected_outcomes": ["navigation_completed"],
                    "safety_considerations": ["avoid_obstacles"]
                }
            ]

    def robot_feedback_callback(self, msg):
        """Handle feedback from robot execution"""
        feedback = json.loads(msg.data)  # Assume feedback is JSON
        current_step = self.current_plan[self.current_step_index] if self.current_plan and self.current_step_index < len(self.current_plan) else None
        
        self.get_logger().info(f"Received feedback: {feedback}")
        
        # Update plan based on feedback
        if current_step and feedback.get("status") == "completed":
            self.current_step_index += 1
            if self.current_step_index < len(self.current_plan):
                # Execute next step
                next_step = self.current_plan[self.current_step_index]
                self.execute_plan_step(next_step)
            else:
                # Plan completed
                self.get_logger().info("Plan completed successfully")
                self.current_plan = []
                self.current_step_index = 0
        elif current_step and feedback.get("status") == "failed":
            # Handle failure - retry, replan, or abort
            self.handle_step_failure(current_step, feedback)

    def execute_plan_step(self, step):
        """Execute a single plan step"""
        self.get_logger().info(f"Executing step {step['step_id']}: {step['action']}")
        
        # Publish step for execution by robot
        step_msg = String()
        step_msg.data = json.dumps(step)
        
        # Determine appropriate topic based on action type
        action_topic = f"/{step['action']}_command"
        # In a real implementation, you would publish to specific action servers
        
        # For now, log the action
        self.get_logger().info(f"Would publish step to {action_topic}: {step}")

    def handle_step_failure(self, step, feedback):
        """Handle plan step failure"""
        self.get_logger().warn(f"Step {step['step_id']} failed: {feedback}")
        
        # Determine if we can recover or need to replan
        error_type = feedback.get("error_type", "unknown")
        
        if error_type == "object_not_found":
            # Try to replan for object detection
            self.replan_object_detection(step)
        elif error_type == "navigation_failed":
            # Try alternative navigation path
            self.replan_navigation(step)
        elif error_type == "grasp_failed":
            # Try different grasp pose or approach
            self.retry_grasp(step)
        else:
            # For other errors, consider aborting or requesting human assistance
            self.get_logger().error("Unknown error type, aborting plan")
            self.current_plan = []
            self.current_step_index = 0

    def replan_object_detection(self, failed_step):
        """Replan for object detection failure"""
        self.get_logger().info("Attempting to replan for object detection...")
        
        # This would involve:
        # 1. Requesting updated environment information
        # 2. Possibly asking for human assistance
        # 3. Trying a different detection strategy
        pass

    def replan_navigation(self, failed_step):
        """Replan for navigation failure"""
        self.get_logger().info("Attempting to replan navigation...")
        
        # This would involve:
        # 1. Updating map with new obstacle information
        # 2. Computing alternative path
        # 3. Continuing with modified plan
        pass

    def retry_grasp(self, failed_step):
        """Retry grasp with different parameters"""
        self.get_logger().info("Attempting to retry grasp...")
        
        # This would involve:
        # 1. Updating object model with new sensing
        # 2. Computing alternative grasp pose
        # 3. Retrying grasp action
        pass


def main(args=None):
    rclpy.init(args=args)
    planner = LLMBasedCognitivePlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Context-Aware Planning

### Implementing Context Awareness:

```python
class ContextAwareCognitivePlanner(LLMBasedCognitivePlanner):
    def __init__(self):
        super().__init__()
        
        # Subscribe to additional context sources
        self.environment_sub = self.create_subscription(
            String, '/environment_map', self.update_environment_context, 10)
        
        self.object_detection_sub = self.create_subscription(
            String, '/detected_objects', self.update_object_context, 10)
        
        self.robot_state_sub = self.create_subscription(
            String, '/robot_state', self.update_robot_state_context, 10)
        
        # Initialize context with default values
        self.planning_context["last_updated"] = self.get_clock().now().nanoseconds
        self.planning_context["knowledge_trust_score"] = 0.9  # Initial trust in knowledge

    def update_environment_context(self, msg):
        """Update environment context"""
        try:
            env_data = json.loads(msg.data)
            self.planning_context["environment_map"] = env_data
            self.planning_context["last_updated"] = self.get_clock().now().nanoseconds
            self.get_logger().info("Environment context updated")
        except Exception as e:
            self.get_logger().error(f"Error updating environment context: {e}")

    def update_object_context(self, msg):
        """Update object location context"""
        try:
            objects_data = json.loads(msg.data)
            self.planning_context["object_locations"] = objects_data
            self.planning_context["last_updated"] = self.get_clock().now().nanoseconds
            self.get_logger().info(f"Object context updated with {len(objects_data)} objects")
        except Exception as e:
            self.get_logger().error(f"Error updating object context: {e}")

    def update_robot_state_context(self, msg):
        """Update robot state context"""
        try:
            state_data = json.loads(msg.data)
            self.planning_context["robot_state"] = state_data
            self.planning_context["last_updated"] = self.get_clock().now().nanoseconds
            self.get_logger().info("Robot state context updated")
        except Exception as e:
            self.get_logger().error(f"Error updating robot state context: {e}")

    def generate_plan_with_context(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Generate plan with enhanced context awareness"""
        # Enhance context with temporal information
        current_time = self.get_clock().now().nanoseconds
        time_since_update = current_time - self.planning_context["last_updated"]
        
        # Adjust trust score based on context freshness
        if time_since_update > 60 * 1e9:  # 60 seconds in nanoseconds
            self.planning_context["knowledge_trust_score"] = 0.5
        elif time_since_update > 5 * 1e9:  # 5 seconds
            self.planning_context["knowledge_trust_score"] = 0.8
        
        # Create enhanced prompt with context
        prompt = self.create_context_enhanced_prompt(command)
        
        # In a real implementation, call LLM with enhanced context
        # For now, return the mock plan as before
        return self.mock_plan_generation(command)

    def create_context_enhanced_prompt(self, command: str) -> str:
        """Create a prompt that includes rich contextual information"""
        # This would create a comprehensive prompt incorporating all context
        return f"""
        Command: {command}
        Context: {json.dumps(self.planning_context, indent=2)}
        Generate appropriate plan.
        """
```

## Hierarchical Task Decomposition

### Multi-Level Planning Architecture:

```python
from enum import Enum

class TaskLevel(Enum):
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"

class HierarchicalCognitivePlanner(ContextAwareCognitivePlanner):
    def __init__(self):
        super().__init__()
        
        # Maintain plans at different levels
        self.high_level_plan = []
        self.medium_level_plan = []
        self.low_level_plan = []
        
        self.current_level = TaskLevel.HIGH

    def generate_plan_with_hierarchy(self, command: str) -> Dict[TaskLevel, List[Dict[str, Any]]]:
        """Generate plan at multiple levels of abstraction"""
        high_level_plan = self.generate_high_level_plan(command)
        
        if not high_level_plan:
            return {}
        
        # Decompose high-level plan into medium-level tasks
        medium_level_plan = []
        for high_task in high_level_plan:
            medium_tasks = self.decompose_to_medium_level(high_task)
            medium_level_plan.extend(medium_tasks)
        
        # Decompose medium-level plan into low-level tasks
        low_level_plan = []
        for medium_task in medium_level_plan:
            low_tasks = self.decompose_to_low_level(medium_task)
            low_level_plan.extend(low_tasks)
        
        return {
            TaskLevel.HIGH: high_level_plan,
            TaskLevel.MEDIUM: medium_level_plan,
            TaskLevel.LOW: low_level_plan
        }

    def generate_high_level_plan(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Generate high-level strategic plan"""
        # High-level plan focuses on major objectives
        # Example: "Bring coffee" -> ["navigate_to_kitchen", "get_container", "get_coffee", "return_to_user"]
        
        high_level_prompt = f"""
        For the command "{command}", create a high-level plan focusing on major objectives.
        Each objective should be achievable in one or two medium-level tasks.
        
        Return as JSON array with structure:
        {{
            "task_id": string,
            "objective": string,
            "priority": integer,
            "estimated_duration": float  # in minutes
        }}
        """
        
        # For demonstration, return a mock high-level plan
        if "coffee" in command.lower():
            return [
                {
                    "task_id": "HL001",
                    "objective": "Navigate to kitchen",
                    "priority": 1,
                    "estimated_duration": 0.5
                },
                {
                    "task_id": "HL002",
                    "objective": "Get coffee container",
                    "priority": 2,
                    "estimated_duration": 1.0
                },
                {
                    "task_id": "HL003",
                    "objective": "Fill container with coffee",
                    "priority": 3,
                    "estimated_duration": 2.0
                },
                {
                    "task_id": "HL004",
                    "objective": "Return to user",
                    "priority": 4,
                    "estimated_duration": 0.5
                }
            ]
        else:
            # Default for other commands
            return [
                {
                    "task_id": "HL001",
                    "objective": "Understand command and plan accordingly",
                    "priority": 1,
                    "estimated_duration": 0.1
                }
            ]

    def decompose_to_medium_level(self, high_task: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Decompose high-level task into medium-level actions"""
        # Medium-level tasks are more specific but still high-level
        # Example: "Get coffee container" -> ["navigate_to_cabinet", "detect_cup", "grasp_cup"]
        
        objective = high_task["objective"]
        
        if "navigate" in objective.lower():
            return [
                {
                    "task_id": f"ML{high_task['task_id']}_01",
                    "action": "navigate",
                    "description": objective,
                    "parameters": {"target_location": self.extract_location(objective)}
                }
            ]
        elif "get container" in objective.lower():
            return [
                {
                    "task_id": f"ML{high_task['task_id']}_01",
                    "action": "navigate",
                    "description": "Navigate to container location",
                    "parameters": {"target_location": "kitchen_cabinet"}
                },
                {
                    "task_id": f"ML{high_task['task_id']}_02",
                    "action": "detect_object",
                    "description": "Detect coffee container",
                    "parameters": {"object_type": "cup", "color": "any"}
                },
                {
                    "task_id": f"ML{high_task['task_id']}_03",
                    "action": "grasp",
                    "description": "Grasp the container",
                    "parameters": {"object_id": "detected_container"}
                }
            ]
        elif "fill container" in objective.lower():
            return [
                {
                    "task_id": f"ML{high_task['task_id']}_01",
                    "action": "navigate",
                    "description": "Navigate to coffee maker",
                    "parameters": {"target_location": "coffee_maker"}
                },
                {
                    "task_id": f"ML{high_task['task_id']}_02",
                    "action": "operate_appliance",
                    "description": "Operate coffee maker",
                    "parameters": {"appliance": "coffee_maker", "action": "fill_container"}
                }
            ]
        elif "return to user" in objective.lower():
            return [
                {
                    "task_id": f"ML{high_task['task_id']}_01",
                    "action": "localize_user",
                    "description": "Locate user position",
                    "parameters": {}
                },
                {
                    "task_id": f"ML{high_task['task_id']}_02",
                    "action": "navigate",
                    "description": "Navigate to user",
                    "parameters": {"target_location": "user_position"}
                },
                {
                    "task_id": f"ML{high_task['task_id']}_03",
                    "action": "present_object",
                    "description": "Present coffee to user",
                    "parameters": {"object": "coffee_container"}
                }
            ]
        else:
            return []

    def decompose_to_low_level(self, medium_task: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Decompose medium-level task into low-level robot commands"""
        # Low-level tasks are specific robot actions
        # Example: "navigate to kitchen" -> ["set_navigation_goal", "monitor_progress", "arrive_at_waypoint"]
        
        action = medium_task["action"]
        
        if action == "navigate":
            return self.create_navigation_primitives(medium_task)
        elif action == "detect_object":
            return self.create_detection_primitives(medium_task)
        elif action == "grasp":
            return self.create_manipulation_primitives(medium_task)
        else:
            # For other actions, create appropriate low-level primitives
            return [
                {
                    "task_id": f"LL{medium_task['task_id']}_01",
                    "primitive": "execute_action",
                    "command": action,
                    "parameters": medium_task["parameters"]
                }
            ]

    def create_navigation_primitives(self, task: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create low-level navigation primitives"""
        location = task["parameters"]["target_location"]
        
        return [
            {
                "task_id": f"LL{task['task_id']}_01",
                "primitive": "set_navigation_goal",
                "command": "nav2_msgs/action/NavigateToPose",
                "parameters": {"pose": self.get_pose_for_location(location)}
            },
            {
                "task_id": f"LL{task['task_id']}_02",
                "primitive": "monitor_navigation",
                "command": "check_progress",
                "parameters": {"timeout": 60.0}
            },
            {
                "task_id": f"LL{task['task_id']}_03",
                "primitive": "confirm_arrival",
                "command": "verify_pose",
                "parameters": {"tolerance": 0.2}
            }
        ]

    def create_detection_primitives(self, task: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create low-level object detection primitives"""
        obj_type = task["parameters"]["object_type"]
        
        return [
            {
                "task_id": f"LL{task['task_id']}_01",
                "primitive": "enable_perception",
                "command": "start_camera",
                "parameters": {"sensor": "front_camera"}
            },
            {
                "task_id": f"LL{task['task_id']}_02",
                "primitive": "detect_objects",
                "command": "run_object_detection",
                "parameters": {"object_type": obj_type}
            },
            {
                "task_id": f"LL{task['task_id']}_03",
                "primitive": "process_detection",
                "command": "filter_detections",
                "parameters": {"min_confidence": 0.7}
            },
            {
                "task_id": f"LL{task['task_id']}_04",
                "primitive": "store_result",
                "command": "update_object_map",
                "parameters": {"object_type": obj_type}
            }
        ]

    def create_manipulation_primitives(self, task: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Create low-level manipulation primitives"""
        obj_id = task["parameters"]["object_id"]
        
        return [
            {
                "task_id": f"LL{task['task_id']}_01",
                "primitive": "plan_grasp",
                "command": "compute_grasp_pose",
                "parameters": {"object_id": obj_id}
            },
            {
                "task_id": f"LL{task['task_id']}_02",
                "primitive": "approach_object",
                "command": "move_to_approach_pose",
                "parameters": {"speed": 0.1}
            },
            {
                "task_id": f"LL{task['task_id']}_03",
                "primitive": "execute_grasp",
                "command": "close_gripper",
                "parameters": {"force": 20.0}
            },
            {
                "task_id": f"LL{task['task_id']}_04",
                "primitive": "lift_object",
                "command": "move_to_transport_pose",
                "parameters": {"height": 0.2}
            }
        ]

    def extract_location(self, objective: str) -> str:
        """Extract location from objective description"""
        # Simple extraction - would use NLP in real implementation
        if "kitchen" in objective.lower():
            return "kitchen"
        elif "living room" in objective.lower():
            return "living_room"
        else:
            return "default"  # Fallback location

    def get_pose_for_location(self, location: str) -> Dict[str, float]:
        """Get predefined pose for a location"""
        # In a real system, this would come from a map or localization system
        locations = {
            "kitchen": {"x": 1.0, "y": 2.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            "living_room": {"x": -1.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
            "default": {"x": 0.0, "y": 0.0, "z": 0.0, "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
        }
        return locations.get(location, locations["default"])
```

## Plan Validation and Safety

### Implementing Plan Validation:

```python
class SafeCognitivePlanner(HierarchicalCognitivePlanner):
    def __init__(self):
        super().__init__()
        
        # Safety validators
        self.safety_validators = [
            self.validate_navigation_safety,
            self.validate_manipulation_safety,
            self.validate_environment_safety
        ]

    def generate_plan_with_validation(self, command: str) -> Optional[List[Dict[str, Any]]]:
        """Generate plan and validate safety before execution"""
        # Generate the plan
        plan = self.generate_plan_with_context(command)
        
        if not plan:
            return None
        
        # Validate the entire plan
        if self.validate_plan(plan):
            self.get_logger().info("Plan validation passed")
            return plan
        else:
            self.get_logger().warn("Plan validation failed")
            # Try to generate a safer alternative plan
            return self.generate_safe_alternative_plan(command, plan)

    def validate_plan(self, plan: List[Dict[str, Any]]) -> bool:
        """Validate plan against safety constraints"""
        for step in plan:
            for validator in self.safety_validators:
                if not validator(step):
                    return False
        return True

    def validate_navigation_safety(self, step: Dict[str, Any]) -> bool:
        """Validate navigation safety"""
        if step.get("action") != "navigate":
            return True  # Only validate navigation steps
        
        # Check if target location is known and safe
        target_location = step["parameters"].get("target_location", "")
        
        if target_location in ["unknown", "forbidden"]:
            self.get_logger().warn(f"Navigation to unknown/forbidden location: {target_location}")
            return False
        
        # Check for path feasibility
        # Check if path is obstacle-free (would use costmap in real implementation)
        
        return True

    def validate_manipulation_safety(self, step: Dict[str, Any]) -> bool:
        """Validate manipulation safety"""
        if step.get("action") not in ["grasp", "place", "manipulate"]:
            return True
        
        # Check if target object is safe to manipulate
        obj_type = step["parameters"].get("object_type", "").lower()
        
        # Don't manipulate dangerous objects
        dangerous_objects = ["knife", "hot", "sharp", "fragile"]  # Simplified safety check
        if any(danger in obj_type for danger in dangerous_objects):
            self.get_logger().warn(f"Attempt to manipulate potentially dangerous object: {obj_type}")
            return False
        
        # Check if robot can safely handle object weight
        if "object_weight" in step["parameters"]:
            obj_weight = step["parameters"]["object_weight"]
            if obj_weight > self.planning_context["robot_capabilities"]["payload"]:
                self.get_logger().warn(f"Object too heavy: {obj_weight}kg vs {self.planning_context['robot_capabilities']['payload']}kg limit")
                return False
        
        return True

    def validate_environment_safety(self, step: Dict[str, Any]) -> bool:
        """Validate environment safety"""
        # Check for environmental hazards
        # Check if robot is in safe state for this action
        # Verify that preconditions are met
        
        # For navigation, check if robot is currently in a safe state
        if step.get("action") == "navigate":
            if self.planning_context.get("robot_state", {}).get("emergency_stop", False):
                self.get_logger().warn("Robot in emergency stop state, cannot navigate")
                return False
        
        return True

    def generate_safe_alternative_plan(self, command: str, original_plan: List[Dict[str, Any]]) -> Optional[List[Dict[str, Any]]]:
        """Generate a safer alternative plan"""
        # In a real implementation, this would try to modify the plan
        # to make it safer while still achieving the goal
        
        self.get_logger().info("Generating safer alternative plan...")
        
        # For now, return a simplified safe plan
        return [
            {
                "step_id": 1,
                "action": "request_human_assistance",
                "description": f"Request human assistance for: {command}",
                "parameters": {"command": command},
                "preconditions": ["human_available"],
                "expected_outcomes": ["task_completed_by_human"],
                "safety_considerations": ["ensure_human_safety", "maintain_communication"]
            }
        ]

    def execute_plan_with_monitoring(self, plan: List[Dict[str, Any]]):
        """Execute plan with continuous safety monitoring"""
        for i, step in enumerate(plan):
            # Monitor safety before executing each step
            if not self.check_current_safety_state():
                self.get_logger().error("Safety check failed, stopping execution")
                return
            
            # Execute step
            self.execute_single_step(step)
            
            # Wait for feedback
            if not self.wait_for_step_completion(step, timeout=60.0):
                self.get_logger().error(f"Step {step['step_id']} timed out")
                # Handle timeout - maybe abort or retry
                return

    def check_current_safety_state(self) -> bool:
        """Check if it's safe to continue executing plan"""
        # Check robot state
        robot_state = self.planning_context.get("robot_state", {})
        
        # Check for emergency stops
        if robot_state.get("emergency_stop", False):
            return False
        
        # Check for safety system warnings
        if robot_state.get("safety_warning", False):
            return False
        
        # Check battery level
        battery_level = robot_state.get("battery_level", 100)
        if battery_level < 20:  # Low battery
            return False
        
        return True
```

## Handling Ambiguity and Uncertainty

### Uncertainty Management in Cognitive Planning:

```python
class UncertaintyAwarePlanner(SafeCognitivePlanner):
    def __init__(self):
        super().__init__()
        
        # Maintain uncertainty models
        self.uncertainty_models = {
            "object_locations": {},  # Uncertainty in object positions
            "environment_map": {},   # Uncertainty in environment
            "robot_state": {}        # Uncertainty in robot state
        }

    def handle_ambiguous_command(self, command: str) -> List[Dict[str, Any]]:
        """Handle ambiguous commands requiring clarification"""
        # Detect ambiguity in command
        if self.is_command_ambiguous(command):
            # Generate clarification questions
            questions = self.generate_clarification_questions(command)
            
            # Return questions to user interface
            return self.create_clarification_plan(questions)
        
        # If not ambiguous, proceed with normal planning
        return self.generate_plan_with_validation(command)

    def is_command_ambiguous(self, command: str) -> bool:
        """Check if command contains ambiguous elements"""
        # Look for ambiguous references
        ambiguous_indicators = [
            r"\bthe (object|thing|item)\b",  # Unspecified objects
            r"\bthat\b",                   # Ambiguous reference
            r"\bthere\b",                  # Unclear location
            r"\bthis\b",                   # Ambiguous reference
        ]
        
        import re
        for pattern in ambiguous_indicators:
            if re.search(pattern, command, re.IGNORECASE):
                return True
        
        return False

    def generate_clarification_questions(self, command: str) -> List[str]:
        """Generate questions to clarify ambiguous command"""
        questions = []
        
        # Based on the command, generate appropriate questions
        if "the" in command and "cup" in command:
            questions.append("Which cup would you like me to bring? I can see a red cup, a blue cup, and a white cup.")
        elif "there" in command:
            questions.append("Could you point to or be more specific about where you mean? I don't have enough information about your reference.")
        elif "that" in command:
            questions.append("Could you be more specific about which object you're referring to?")
        
        return questions if questions else [f"Could you clarify what you mean by: '{command}'?"]

    def create_clarification_plan(self, questions: List[str]) -> List[Dict[str, Any]]:
        """Create plan to request clarification from user"""
        return [
            {
                "step_id": 1,
                "action": "speak_request",
                "description": "Ask for clarification",
                "parameters": {"questions": questions},
                "preconditions": ["robot_can_speak"],
                "expected_outcomes": ["user_response_received", "command_resolved"],
                "safety_considerations": ["wait_for_response", "timeout_handling"]
            },
            {
                "step_id": 2,
                "action": "wait_for_response",
                "description": "Wait for user to clarify",
                "parameters": {"timeout": 30.0},
                "preconditions": ["request_made"],
                "expected_outcomes": ["clarification_received"],
                "safety_considerations": ["timeout_handling"]
            }
        ]

    def incorporate_uncertainty(self, plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Incorporate uncertainty into plan"""
        # For each step, add uncertainty-aware components
        enhanced_plan = []
        for step in plan:
            # Add confidence thresholds
            step["confidence_required"] = step.get("confidence_required", 0.9)
            
            # Add verification steps
            verification_step = {
                "step_id": f"{step['step_id']}v",
                "action": "verify",
                "description": f"Verify outcome of step {step['step_id']}",
                "parameters": step.get("expected_outcomes", []),
                "preconditions": [f"step_{step['step_id']}_completed"],
                "expected_outcomes": ["verification_passed"],
                "safety_considerations": ["abort_if_unverified"]
            }
            
            enhanced_plan.extend([step, verification_step])
        
        return enhanced_plan
```

## Evaluation Metrics

### Cognitive Planning Performance Metrics:

```python
class CognitivePlanningMetrics:
    def __init__(self):
        self.metrics = {
            'plan_success_rate': 0.0,
            'task_completion_time': 0.0,
            'planning_time': 0.0,
            'replan_count': 0,
            'human_intervention_rate': 0.0,
            'safety_violations': 0,
            'ambiguity_resolution_rate': 0.0,
            'knowledge_utilization_score': 0.0
        }
        
        self.plan_attempts = 0
        self.plan_successes = 0
        self.task_times = []
        self.planning_times = []
        self.replan_counts = []
        self.human_interventions = 0
        self.total_interactions = 0
        self.safety_violations = 0
        self.ambiguous_commands = 0
        self.resolved_commands = 0

    def record_plan_attempt(self, success: bool, planning_time: float = None, task_time: float = None):
        """Record metrics for a planning attempt"""
        self.plan_attempts += 1
        if success:
            self.plan_successes += 1
            if task_time is not None:
                self.task_times.append(task_time)
        
        if planning_time is not None:
            self.planning_times.append(planning_time)
        
        # Update metrics
        self.metrics['plan_success_rate'] = self.plan_successes / self.plan_attempts if self.plan_attempts > 0 else 0
        self.metrics['planning_time'] = sum(self.planning_times) / len(self.planning_times) if self.planning_times else 0
        self.metrics['task_completion_time'] = sum(self.task_times) / len(self.task_times) if self.task_times else 0

    def record_replan_event(self, reason: str):
        """Record when a plan needs to be regenerated"""
        self.replan_counts.append(reason)

    def record_human_intervention(self):
        """Record when human assistance is required"""
        self.human_interventions += 1
        self.total_interactions += 1

        self.metrics['human_intervention_rate'] = self.human_interventions / self.total_interactions if self.total_interactions > 0 else 0

    def record_safety_violation(self):
        """Record safety-related issues"""
        self.safety_violations += 1
        self.metrics['safety_violations'] = self.safety_violations

    def record_ambiguity_handling(self, command: str, resolved: bool):
        """Record ambiguity detection and resolution"""
        self.ambiguous_commands += 1
        if resolved:
            self.resolved_commands += 1
        
        self.metrics['ambiguity_resolution_rate'] = self.resolved_commands / self.ambiguous_commands if self.ambiguous_commands > 0 else 0

    def generate_report(self):
        """Generate comprehensive performance report"""
        report = f"""
        Cognitive Planning Performance Report:
        - Plan Success Rate: {self.metrics['plan_success_rate']:.2%}
        - Avg Task Completion Time: {self.metrics['task_completion_time']:.2f}s
        - Avg Planning Time: {self.metrics['planning_time']:.2f}s
        - Replan Count: {len(self.replan_counts)}
        - Human Intervention Rate: {self.metrics['human_intervention_rate']:.2%}
        - Safety Violations: {self.metrics['safety_violations']}
        - Ambiguity Resolution Rate: {self.metrics['ambiguity_resolution_rate']:.2%}
        """
        return report
```

## Best Practices for LLM Cognitive Planning

### 1. Planning Strategies:
- Use hierarchical planning for complex tasks
- Implement plan validation and safety checks
- Provide fallback mechanisms for plan failures
- Maintain plan flexibility for real-world changes

### 2. Context Management:
- Keep context updated with real-time information
- Manage context size to optimize LLM performance
- Implement context fading for outdated information
- Integrate with robot's sensory and state information

### 3. Error Handling:
- Implement robust error detection and recovery
- Use multiple planning approaches for redundancy
- Provide clear feedback when plans fail
- Allow for human intervention when needed

## Summary

This chapter explored the implementation of cognitive planning using Large Language Models for robotics. We covered hierarchical planning architectures, context-aware planning, plan validation, and uncertainty management.

LLM-based cognitive planning enables robots to understand and execute high-level, natural language commands by decomposing them into sequences of executable actions. The approach combines the power of LLMs' natural language understanding and world knowledge with the precision of robotic execution.

Key challenges include grounding LLM knowledge in the physical world, handling ambiguity in human commands, ensuring plan safety, and managing the uncertainty inherent in both language understanding and robot execution.

## Exercises

1. Implement an LLM-based cognitive planner for a specific robot task in simulation.
2. Develop a hierarchical planning system that decomposes tasks at multiple levels.
3. Create safety validation mechanisms for LLM-generated plans.
4. Implement ambiguity detection and resolution for unclear commands.
5. Evaluate the performance of your cognitive planning system with various natural language commands.