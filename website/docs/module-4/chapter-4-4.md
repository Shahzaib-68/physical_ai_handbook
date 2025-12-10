---
sidebar_position: 19
slug: /module-4/chapter-4-4
title: Chapter 4.4 - Natural Language to ROS 2 Actions
---

# Chapter 4.4: Natural Language to ROS 2 Actions

## Overview

Converting natural language commands to executable ROS 2 actions is a crucial capability for human-robot interaction. This chapter explores techniques for parsing natural language, mapping it to ROS 2 services, topics, and actions, and executing robot behaviors based on human instructions. We'll implement complete pipelines for translating high-level commands into low-level robot operations.

## Learning Objectives

By the end of this chapter, students will be able to:
- Parse natural language commands into structured robot actions
- Map language concepts to ROS 2 service calls, topics, and actions
- Implement semantic parsing for robotics tasks
- Design natural language interfaces for robot control
- Handle ambiguous or incomplete language commands
- Create robust natural language to action conversion systems

## Introduction to Natural Language to ROS 2 Actions

The conversion of natural language to ROS 2 actions involves multiple components working together:
1. **Language Understanding**: Converting natural language to intermediate representations
2. **Action Mapping**: Converting representations to specific ROS 2 operations
3. **Parameter Extraction**: Identifying target objects, locations, and values
4. **Execution**: Calling ROS 2 services, publishing to topics, or executing actions

### ROS 2 Action Types:
- **Services**: Request-response operations (e.g., `SetBool`, `GetMap`)
- **Topics**: Continuous data streams (e.g., `cmd_vel`, `joint_states`)
- **Actions**: Long-running operations with feedback (e.g., `NavigateToPose`, `FollowJointTrajectory`)

## Architecture for Language to Actions

### System Architecture:

```
[Human Command] → [NLP Parser] → [Action Mapper] → [ROS 2 Client] → [Robot Execution]
      ↓              ↓              ↓              ↓
[Context] ← [Intent Extraction] ← [Validation] ← [Feedback]
```

### Core Components:
1. **Command Parser**: Identifies intent and entities in natural language
2. **Action Mapper**: Maps intents to ROS 2 operations
3. **Parameter Extractor**: Extracts specific values from commands
4. **ROS 2 Client**: Executes the mapped operations
5. **Context Manager**: Maintains task and environment context

## Implementing Natural Language Parser

### Basic Natural Language to ROS 2 Node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import json
import re
from typing import Dict, List, Optional, Tuple, Any


class NaturalLanguageToROS2(Node):
    def __init__(self):
        super().__init__('natural_language_to_ros2')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.nav_goal_pub = self.create_publisher(Pose, '/goal_pose', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/natural_command', self.command_callback, 10)
        self.feedback_pub = self.create_publisher(String, '/nl_feedback', 10)
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.traj_client = ActionClient(self, FollowJointTrajectory, 'joint_trajectory_controller/follow_joint_trajectory')
        
        # Command parser
        self.command_parser = CommandParser()
        
        # Context manager
        self.context_manager = ContextManager(self)
        
        self.get_logger().info("Natural Language to ROS 2 Interface Initialized")

    def command_callback(self, msg):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f"Processing command: {command}")
        
        # Parse command
        parsed_command = self.command_parser.parse_command(command)
        
        if not parsed_command:
            self.get_logger().warn(f"Could not parse command: {command}")
            self.publish_feedback(f"Sorry, I didn't understand: {command}")
            return
        
        # Map to ROS 2 action
        success = self.execute_ros2_action(parsed_command)
        
        if success:
            self.get_logger().info(f"Successfully executed command: {command}")
            self.publish_feedback(f"Done: {command}")
        else:
            self.get_logger().error(f"Failed to execute command: {command}")
            self.publish_feedback(f"Failed to execute: {command}")

    def execute_ros2_action(self, parsed_command: Dict[str, Any]) -> bool:
        """Execute ROS 2 action based on parsed command"""
        action_type = parsed_command.get('action', '').lower()
        
        if action_type == 'navigation':
            return self.execute_navigation(parsed_command)
        elif action_type == 'manipulation':
            return self.execute_manipulation(parsed_command)
        elif action_type == 'movement':
            return self.execute_movement(parsed_command)
        elif action_type == 'speak':
            return self.execute_speak(parsed_command)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
            return False

    def execute_navigation(self, cmd: Dict[str, Any]) -> bool:
        """Execute navigation command"""
        try:
            # Extract navigation parameters
            location = cmd.get('location', 'unknown')
            
            # Use predefined locations or parse coordinates
            pose = self.get_pose_for_location(location)
            
            if not pose:
                self.get_logger().warn(f"Unknown location: {location}")
                return False
            
            # Send navigation goal via action
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose = pose
            
            # Wait for action server
            if not self.nav_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error("Navigation action server not available")
                return False
            
            # Send goal
            future = self.nav_client.send_goal_async(goal_msg)
            future.add_done_callback(self.navigation_done_callback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error executing navigation: {e}")
            return False

    def execute_manipulation(self, cmd: Dict[str, Any]) -> bool:
        """Execute manipulation command"""
        try:
            action = cmd.get('manipulation_action', '')
            
            if action == 'grasp' or action == 'pick_up':
                return self.execute_grasp(cmd)
            elif action == 'place' or action == 'put_down':
                return self.execute_place(cmd)
            elif action == 'move_arm':
                return self.execute_arm_movement(cmd)
            else:
                self.get_logger().warn(f"Unknown manipulation action: {action}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error executing manipulation: {e}")
            return False

    def execute_grasp(self, cmd: Dict[str, Any]) -> bool:
        """Execute grasping action"""
        # Get object information
        obj_name = cmd.get('target_object', 'unknown')
        
        # Get object pose (would come from perception system)
        obj_pose = self.get_object_pose(obj_name)
        
        if not obj_pose:
            self.get_logger().warn(f"Could not find object: {obj_name}")
            return False
        
        # Plan and execute grasp (simplified)
        grasp_poses = self.plan_grasp_poses(obj_pose)
        
        for grasp_pose in grasp_poses:
            if self.execute_approach_and_grasp(grasp_pose):
                self.get_logger().info(f"Successfully grasped {obj_name}")
                return True
        
        self.get_logger().warn(f"Failed to grasp {obj_name}")
        return False

    def execute_movement(self, cmd: Dict[str, Any]) -> bool:
        """Execute basic movement commands"""
        try:
            direction = cmd.get('direction', 'forward')
            speed = cmd.get('speed', 0.5)
            
            twist_msg = Twist()
            
            if direction == 'forward':
                twist_msg.linear.x = speed
            elif direction == 'backward':
                twist_msg.linear.x = -speed
            elif direction == 'left':
                twist_msg.linear.y = speed
            elif direction == 'right':
                twist_msg.linear.y = -speed
            elif direction == 'rotate_left':
                twist_msg.angular.z = speed
            elif direction == 'rotate_right':
                twist_msg.angular.z = -speed
            else:
                self.get_logger().warn(f"Unknown direction: {direction}")
                return False
            
            self.cmd_vel_pub.publish(twist_msg)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error executing movement: {e}")
            return False

    def execute_speak(self, cmd: Dict[str, Any]) -> bool:
        """Execute speech command"""
        try:
            text = cmd.get('text', 'Hello!')
            
            # Publish to speech system (would be connected to TTS)
            speech_msg = String()
            speech_msg.data = text
            
            # Publish to speech topic
            # self.speech_pub.publish(speech_msg)
            
            self.get_logger().info(f"Speaking: {text}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error executing speech: {e}")
            return False

    def get_pose_for_location(self, location: str) -> Optional[Pose]:
        """Get predefined pose for a location"""
        # In a real system, this would come from a map or localization system
        location_poses = {
            'kitchen': Pose(position={'x': 2.0, 'y': 1.0, 'z': 0.0},
                           orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}),
            'living_room': Pose(position={'x': -1.0, 'y': 0.0, 'z': 0.0},
                              orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}),
            'bedroom': Pose(position={'x': 3.0, 'y': -2.0, 'z': 0.0},
                           orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}),
            'office': Pose(position={'x': 0.0, 'y': 2.0, 'z': 0.0},
                          orientation={'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
        }
        
        # Convert dict keys to proper format for Pose
        pose_dict = location_poses.get(location.lower().replace(' ', '_'))
        if pose_dict:
            pose = Pose()
            pose.position.x = pose_dict.position['x']
            pose.position.y = pose_dict.position['y']
            pose.position.z = pose_dict.position['z']
            pose.orientation.x = pose_dict.orientation['x']
            pose.orientation.y = pose_dict.orientation['y']
            pose.orientation.z = pose_dict.orientation['z']
            pose.orientation.w = pose_dict.orientation['w']
            return pose
        
        return None

    def get_object_pose(self, obj_name: str) -> Optional[Any]:
        """Get pose of an object (placeholder)"""
        # In a real system, this would come from perception
        return self.context_manager.get_object_pose(obj_name)

    def plan_grasp_poses(self, obj_pose: Any) -> List[Any]:
        """Plan grasp poses for an object (placeholder)"""
        # In a real system, this would compute actual grasp poses
        return [obj_pose]  # Simplified

    def execute_approach_and_grasp(self, grasp_pose: Any) -> bool:
        """Execute approach and grasp (placeholder)"""
        # In a real system, this would control the manipulator
        return True  # Simplified

    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Navigation goal accepted')
            else:
                self.get_logger().warn('Navigation goal rejected')
        except Exception as e:
            self.get_logger().error(f'Navigation failed: {e}')

    def publish_feedback(self, message: str):
        """Publish feedback message"""
        feedback_msg = String()
        feedback_msg.data = message
        self.feedback_pub.publish(feedback_msg)


class CommandParser:
    def __init__(self):
        # Define command patterns
        self.patterns = {
            # Navigation patterns
            r'go to (the )?(.+?)$': ('navigation', ['location']),
            r'move to (the )?(.+?)$': ('navigation', ['location']),
            r'navigate to (the )?(.+?)$': ('navigation', ['location']),
            r'go (forward|backward|left|right)$': ('movement', ['direction']),
            r'move (forward|backward|left|right)$': ('movement', ['direction']),
            r'turn (left|right)$': ('movement', ['direction']),
            
            # Manipulation patterns
            r'pick up (the )?(.+?)$': ('manipulation', ['manipulation_action', 'target_object'], ['pick_up', 'obj_name']),
            r'grasp (the )?(.+?)$': ('manipulation', ['manipulation_action', 'target_object'], ['grasp', 'obj_name']),
            r'pickup (the )?(.+?)$': ('manipulation', ['manipulation_action', 'target_object'], ['pick_up', 'obj_name']),
            r'put (the )?(.+?) (down|away|there)$': ('manipulation', ['manipulation_action', 'target_object'], ['place', 'obj_name']),
            
            # General commands
            r'say (.+?)$': ('speak', ['text']),
            r'repeat (.+?)$': ('speak', ['text']),
        }
    
    def parse_command(self, command: str) -> Optional[Dict[str, Any]]:
        """Parse natural language command into structured format"""
        command_lower = command.lower().strip()
        
        for pattern, config in self.patterns.items():
            match = re.match(pattern, command_lower)
            if match:
                if len(config) == 2:
                    # Simple pattern: action, fields
                    action, fields = config
                    result = {'action': action}
                    for i, field in enumerate(fields):
                        if i < len(match.groups()):
                            result[field] = match.group(i + 1)
                else:
                    # Complex pattern: action, fields, values
                    action, fields, values = config
                    result = {'action': action}
                    for field, value in zip(fields, values):
                        if value == 'obj_name':
                            result[field] = match.group(len(fields) - fields.index(field))
                        else:
                            result[field] = value
                
                return result
        
        # If no specific pattern matches, try semantic analysis
        return self.semantic_parse(command_lower)

    def semantic_parse(self, command: str) -> Optional[Dict[str, Any]]:
        """Perform semantic analysis when regex patterns fail"""
        # Simple semantic analysis
        if any(word in command for word in ['go', 'move', 'navigate']):
            # Try to extract location from command
            locations = ['kitchen', 'bedroom', 'living room', 'office', 'bathroom']
            for loc in locations:
                if loc in command:
                    return {'action': 'navigation', 'location': loc.replace(' ', '_')}
        
        if any(word in command for word in ['pick', 'grasp', 'lift']):
            # Extract object
            words = command.split()
            # Simple object extraction
            for i, word in enumerate(words):
                if word in ['the', 'a', 'an'] and i + 1 < len(words):
                    return {
                        'action': 'manipulation', 
                        'manipulation_action': 'pick_up',
                        'target_object': words[i + 1]
                    }
        
        # If still no match, return general command
        return {
            'action': 'general',
            'command_type': 'unknown',
            'raw_command': command
        }


class ContextManager:
    def __init__(self, node: Node):
        self.node = node
        self.object_map = {}  # Map of object names to poses
        self.location_map = {}  # Map of location names to poses
        self.robot_state = {}  # Current robot state
        
        # Subscribe to relevant topics to maintain context
        self.node.create_subscription(
            String, '/detected_objects', self.update_object_map, 10)
    
    def update_object_map(self, msg):
        """Update object map from perception system"""
        try:
            objects = json.loads(msg.data)
            self.object_map.update(objects)
            self.node.get_logger().info(f"Updated object map with {len(objects)} objects")
        except Exception as e:
            self.node.get_logger().error(f"Error updating object map: {e}")
    
    def get_object_pose(self, obj_name: str) -> Optional[Any]:
        """Get pose of an object from context"""
        # Normalize object name
        normalized_name = obj_name.lower().replace(' ', '_').strip()
        
        # Try exact match first
        if normalized_name in self.object_map:
            return self.object_map[normalized_name]
        
        # Try partial match
        for name, pose in self.object_map.items():
            if normalized_name in name or name in normalized_name:
                return pose
        
        return None


def main(args=None):
    rclpy.init(args=args)
    nl_to_ros2 = NaturalLanguageToROS2()
    
    try:
        rclpy.spin(nl_to_ros2)
    except KeyboardInterrupt:
        pass
    finally:
        nl_to_ros2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Natural Language Processing

### Implementing Semantic Role Labeling for Robotics:

```python
class SemanticRoleLabeler:
    def __init__(self):
        # Define semantic roles relevant to robotics
        self.roles = {
            'AGENT': 'The entity performing the action (typically the robot)',
            'THEME': 'The primary object being acted upon',
            'LOCATION': 'The location where the action occurs',
            'SOURCE': 'Where something comes from',
            'DESTINATION': 'Where something goes to',
            'INSTRUMENT': 'The tool used for the action',
            'DIRECTION': 'The direction of movement',
        }
    
    def label_command(self, command: str) -> Dict[str, Any]:
        """Label semantic roles in a command"""
        # Simple rule-based role labeling
        command_lower = command.lower()
        
        # Extract roles
        agent = "robot"  # For now, robot is always the agent
        
        # Find theme (object being acted upon)
        theme = self.extract_theme(command_lower)
        
        # Find location
        location = self.extract_location(command_lower)
        
        # Find direction
        direction = self.extract_direction(command_lower)
        
        # Find destination
        destination = self.extract_destination(command_lower)
        
        return {
            'AGENT': agent,
            'THEME': theme,
            'LOCATION': location,
            'DIRECTION': direction,
            'DESTINATION': destination
        }
    
    def extract_theme(self, command: str) -> Optional[str]:
        """Extract the object being acted upon"""
        patterns = [
            r'pick up (the )?(.+?)$',
            r'grasp (the )?(.+?)$',
            r'take (the )?(.+?)$',
            r'bring me (the )?(.+?)$',
            r'get (the )?(.+?)$',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, command)
            if match:
                return match.group(2)
        
        # Simple object extraction
        words = command.split()
        for i, word in enumerate(words):
            if word in ['the', 'a', 'an'] and i + 1 < len(words):
                # Check if next word is a likely object
                obj_word = words[i + 1]
                if obj_word not in ['to', 'from', 'in', 'on', 'at']:
                    return obj_word
        
        return None
    
    def extract_location(self, command: str) -> Optional[str]:
        """Extract location from command"""
        # Look for location indicators
        location_patterns = [
            r'to (the )?(.+?)$',
            r'in (the )?(.+?)$',
            r'at (the )?(.+?)$',
            r'by (the )?(.+?)$',
        ]
        
        for pattern in location_patterns:
            match = re.search(pattern, command)
            if match:
                potential_loc = match.group(2)
                # Verify it's a location and not an object
                if self.is_location(potential_loc):
                    return potential_loc
        
        return None
    
    def extract_direction(self, command: str) -> Optional[str]:
        """Extract movement direction"""
        directions = ['forward', 'backward', 'left', 'right', 'up', 'down']
        for direction in directions:
            if direction in command:
                return direction
        
        # Check for rotation directions
        rotation_dirs = ['clockwise', 'counterclockwise', 'anticlockwise']
        for rot_dir in rotation_dirs:
            if rot_dir in command:
                return rot_dir
        
        return None
    
    def extract_destination(self, command: str) -> Optional[str]:
        """Extract destination from command"""
        # Similar to location extraction but specifically for destinations
        patterns = [
            r'bring it to (the )?(.+?)$',
            r'take it to (the )?(.+?)$',
            r'put it (in|on|at) (the )?(.+?)$',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, command)
            if match:
                # Return the last captured group (the destination)
                groups = match.groups()
                return groups[-1]
        
        return None
    
    def is_location(self, candidate: str) -> bool:
        """Determine if candidate is likely a location"""
        known_locations = ['kitchen', 'bedroom', 'living room', 'office', 'bathroom', 
                          'dining room', 'hallway', 'entrance', 'table', 'cabinet', 
                          'shelf', 'counter', 'couch', 'chair']
        
        return candidate in known_locations or any(loc in candidate for loc in known_locations)


class AdvancedNaturalLanguageToROS2(NaturalLanguageToROS2):
    def __init__(self):
        super().__init__()
        
        # Initialize semantic role labeler
        self.role_labeler = SemanticRoleLabeler()
        
        # Enhanced command parser
        self.enhanced_parser = EnhancedCommandParser()
    
    def command_callback(self, msg):
        """Enhanced command processing with semantic analysis"""
        command = msg.data
        self.get_logger().info(f"Processing advanced command: {command}")
        
        # Perform semantic role labeling
        roles = self.role_labeler.label_command(command)
        self.get_logger().info(f"Semantic roles: {roles}")
        
        # Parse command with enhanced parser
        parsed_command = self.enhanced_parser.parse_command(command, roles)
        
        if not parsed_command:
            self.get_logger().warn(f"Could not parse command: {command}")
            self.publish_feedback(f"Sorry, I didn't understand: {command}")
            return
        
        # Map to ROS 2 action
        success = self.execute_ros2_action(parsed_command, roles)
        
        if success:
            self.get_logger().info(f"Successfully executed command: {command}")
            self.publish_feedback(f"Done: {command}")
        else:
            self.get_logger().error(f"Failed to execute command: {command}")
            self.publish_feedback(f"Failed to execute: {command}")
    
    def execute_ros2_action(self, parsed_command: Dict[str, Any], roles: Dict[str, Any] = None) -> bool:
        """Execute ROS 2 action with semantic context"""
        action_type = parsed_command.get('action', '').lower()
        
        # Enhance command with semantic roles if available
        if roles:
            parsed_command = self.enhance_with_roles(parsed_command, roles)
        
        if action_type == 'navigation':
            return self.execute_navigation(parsed_command)
        elif action_type == 'manipulation':
            return self.execute_manipulation(parsed_command)
        elif action_type == 'movement':
            return self.execute_movement(parsed_command)
        elif action_type == 'transport':
            return self.execute_transport(parsed_command)
        elif action_type == 'speak':
            return self.execute_speak(parsed_command)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
            return False
    
    def enhance_with_roles(self, command: Dict[str, Any], roles: Dict[str, Any]) -> Dict[str, Any]:
        """Enhance command with semantic role information"""
        enhanced = command.copy()
        
        # Add location information from roles
        if roles.get('LOCATION'):
            enhanced['location'] = enhanced.get('location', roles['LOCATION'])
        
        # Add destination information
        if roles.get('DESTINATION'):
            enhanced['destination'] = enhanced.get('destination', roles['DESTINATION'])
        
        # Add theme (object) information
        if roles.get('THEME'):
            enhanced['target_object'] = enhanced.get('target_object', roles['THEME'])
        
        return enhanced


class EnhancedCommandParser:
    def __init__(self):
        # Extended pattern definitions
        self.patterns = [
            # Transport commands (fetch, bring, carry)
            {
                'pattern': r'(bring|carry|fetch|get) me (the )?(.+?)( from .+?)?( to .+?)?$',
                'mapping': {
                    'action': 'transport',
                    'target_object': 3,  # The object to transport
                    'source': None,     # Will be filled via semantic roles
                    'destination': None # Will be filled via semantic roles
                }
            },
            # Complex navigation commands
            {
                'pattern': r'go (to|toward) (the )?(.+?)( and .+?)?$',
                'mapping': {
                    'action': 'navigation',
                    'location': 3
                }
            },
            # Manipulation with destination
            {
                'pattern': r'(move|take|put|place) (the )?(.+?)( from .+?)? (to|in|on) (the )?(.+?)$',
                'mapping': {
                    'action': 'manipulation_with_transport',
                    'target_object': 3,
                    'destination': 6
                }
            },
            # Conditional commands
            {
                'pattern': r'if (.+?) then (.+?)$',
                'mapping': {
                    'action': 'conditional',
                    'condition': 1,
                    'command': 2
                }
            }
        ]
    
    def parse_command(self, command: str, semantic_roles: Dict[str, Any] = None) -> Optional[Dict[str, Any]]:
        """Parse command with semantic role context"""
        command_lower = command.lower().strip()
        
        # Try pattern matching first
        for pattern_config in self.patterns:
            match = re.match(pattern_config['pattern'], command_lower)
            if match:
                result = pattern_config['mapping'].copy()
                
                # Fill positional arguments
                for key, pos in result.items():
                    if isinstance(pos, int) and pos <= len(match.groups()):
                        result[key] = match.group(pos)
                
                # Enhance with semantic roles if available
                if semantic_roles:
                    result = self.apply_semantic_roles(result, semantic_roles)
                
                return result
        
        # Fallback to basic parsing
        basic_parser = CommandParser()
        return basic_parser.parse_command(command)
    
    def apply_semantic_roles(self, parsed: Dict[str, Any], roles: Dict[str, Any]) -> Dict[str, Any]:
        """Apply semantic roles to enhance parsed command"""
        result = parsed.copy()
        
        # Apply location roles
        if 'location' not in result and roles.get('LOCATION'):
            result['location'] = roles['LOCATION']
        
        # Apply destination roles
        if 'destination' not in result and roles.get('DESTINATION'):
            result['destination'] = roles['DESTINATION']
        
        # Apply theme roles
        if 'target_object' not in result and roles.get('THEME'):
            result['target_object'] = roles['THEME']
        
        # Apply source roles
        if 'source' not in result and roles.get('SOURCE'):
            result['source'] = roles['SOURCE']
        
        return result
```

## ROS 2 Action Mapping

### Implementing Action Mapping System:

```python
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.publisher import Publisher
from typing import Union

class ROS2ActionMapper:
    def __init__(self, node: Node):
        self.node = node
        self.action_clients = {}
        self.service_clients = {}
        self.publishers = {}
        
        # Define action mappings
        self.action_mappings = {
            'navigation': {
                'type': NavigateToPose,
                'name': 'navigate_to_pose',
                'method': self.execute_navigate_to_pose
            },
            'follow_trajectory': {
                'type': FollowJointTrajectory,
                'name': 'joint_trajectory_controller/follow_joint_trajectory',
                'method': self.execute_follow_trajectory
            }
        }
        
        # Define service mappings
        self.service_mappings = {
            'set_led': {
                'type': 'std_srvs/srv/SetBool',
                'name': '/set_led',
                'method': self.execute_set_led
            }
        }
        
        # Define topic mappings
        self.topic_mappings = {
            'cmd_vel': {
                'type': Twist,
                'name': '/cmd_vel',
                'method': self.publish_cmd_vel
            },
            'joint_commands': {
                'type': JointState,
                'name': '/joint_commands',
                'method': self.publish_joint_commands
            }
        }
    
    def get_action_client(self, action_type: str) -> Optional[ActionClient]:
        """Get or create action client"""
        if action_type not in self.action_clients:
            mapping = self.action_mappings.get(action_type)
            if mapping:
                client = ActionClient(self.node, mapping['type'], mapping['name'])
                self.action_clients[action_type] = client
            else:
                self.node.get_logger().warn(f"No mapping found for action type: {action_type}")
                return None
        
        return self.action_clients[action_type]
    
    def get_service_client(self, service_type: str) -> Optional[Client]:
        """Get or create service client"""
        if service_type not in self.service_clients:
            mapping = self.service_mappings.get(service_type)
            if mapping:
                # Create client (simplified - in practice, you'd need the service type)
                client = self.node.create_client(mapping['type'], mapping['name'])
                self.service_clients[service_type] = client
            else:
                self.node.get_logger().warn(f"No mapping found for service type: {service_type}")
                return None
        
        return self.service_clients[service_type]
    
    def get_publisher(self, topic_type: str) -> Optional[Publisher]:
        """Get or create publisher"""
        if topic_type not in self.publishers:
            mapping = self.topic_mappings.get(topic_type)
            if mapping:
                publisher = self.node.create_publisher(mapping['type'], mapping['name'], 10)
                self.publishers[topic_type] = publisher
            else:
                self.node.get_logger().warn(f"No mapping found for topic type: {topic_type}")
                return None
        
        return self.publishers[topic_type]
    
    # Action execution methods
    def execute_navigate_to_pose(self, params: Dict[str, Any]) -> bool:
        """Execute navigation action"""
        try:
            # Get action client
            client = self.get_action_client('navigation')
            if not client:
                return False
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = params.get('frame_id', 'map')
            goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.pose.pose = params['pose']
            
            # Wait for server
            if not client.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().error("Navigation server not available")
                return False
            
            # Send goal
            future = client.send_goal_async(goal_msg)
            future.add_done_callback(self.navigation_done_callback)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error in navigate_to_pose: {e}")
            return False
    
    def execute_follow_trajectory(self, params: Dict[str, Any]) -> bool:
        """Execute trajectory following action"""
        try:
            # Get action client
            client = self.get_action_client('follow_trajectory')
            if not client:
                return False
            
            # Create goal message
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = params['trajectory']
            
            # Wait for server
            if not client.wait_for_server(timeout_sec=1.0):
                self.node.get_logger().error("Trajectory server not available")
                return False
            
            # Send goal
            future = client.send_goal_async(goal_msg)
            future.add_done_callback(self.trajectory_done_callback)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error in follow_trajectory: {e}")
            return False
    
    # Service execution methods
    def execute_set_led(self, params: Dict[str, Any]) -> bool:
        """Execute LED set service"""
        try:
            client = self.get_service_client('set_led')
            if not client:
                return False
            
            # Wait for service
            if not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error("Set LED service not available")
                return False
            
            # Create request
            request = {'data': params.get('state', True)}
            
            # Call service
            future = client.call_async(request)
            future.add_done_callback(self.set_led_done_callback)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error in set_led: {e}")
            return False
    
    # Topic publishing methods
    def publish_cmd_vel(self, params: Dict[str, Any]) -> bool:
        """Publish velocity command"""
        try:
            pub = self.get_publisher('cmd_vel')
            if not pub:
                return False
            
            twist = Twist()
            twist.linear.x = params.get('linear_x', 0.0)
            twist.linear.y = params.get('linear_y', 0.0)
            twist.linear.z = params.get('linear_z', 0.0)
            twist.angular.x = params.get('angular_x', 0.0)
            twist.angular.y = params.get('angular_y', 0.0)
            twist.angular.z = params.get('angular_z', 0.0)
            
            pub.publish(twist)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error publishing cmd_vel: {e}")
            return False
    
    def publish_joint_commands(self, params: Dict[str, Any]) -> bool:
        """Publish joint commands"""
        try:
            pub = self.get_publisher('joint_commands')
            if not pub:
                return False
            
            joint_state = JointState()
            joint_state.name = params.get('joint_names', [])
            joint_state.position = params.get('joint_positions', [])
            joint_state.velocity = params.get('joint_velocities', [])
            joint_state.effort = params.get('joint_efforts', [])
            
            pub.publish(joint_state)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error publishing joint commands: {e}")
            return False
    
    def navigation_done_callback(self, future):
        """Handle navigation completion"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.node.get_logger().info('Navigation goal accepted')
            else:
                self.node.get_logger().warn('Navigation goal rejected')
        except Exception as e:
            self.node.get_logger().error(f'Navigation failed: {e}')
    
    def trajectory_done_callback(self, future):
        """Handle trajectory completion"""
        try:
            goal_handle = future.result()
            if goal_handle.accepted:
                self.node.get_logger().info('Trajectory goal accepted')
            else:
                self.node.get_logger().warn('Trajectory goal rejected')
        except Exception as e:
            self.node.get_logger().error(f'Trajectory failed: {e}')
    
    def set_led_done_callback(self, future):
        """Handle LED service completion"""
        try:
            response = future.result()
            self.node.get_logger().info(f'LED set result: {response.success}')
        except Exception as e:
            self.node.get_logger().error(f'LED service failed: {e}')


class MappedNaturalLanguageToROS2(AdvancedNaturalLanguageToROS2):
    def __init__(self):
        super().__init__()
        
        # Initialize action mapper
        self.action_mapper = ROS2ActionMapper(self)
    
    def execute_ros2_action(self, parsed_command: Dict[str, Any], roles: Dict[str, Any] = None) -> bool:
        """Execute action using the action mapper"""
        action_type = parsed_command.get('action', '').lower()
        
        # Define execution methods for different action types
        execution_methods = {
            'navigation': self.execute_mapped_navigation,
            'manipulation': self.execute_mapped_manipulation,
            'movement': self.execute_mapped_movement,
            'transport': self.execute_mapped_transport,
            'speak': self.execute_speak,
        }
        
        method = execution_methods.get(action_type)
        if method:
            return method(parsed_command)
        else:
            self.get_logger().warn(f"Unknown action type: {action_type}")
            return False
    
    def execute_mapped_navigation(self, cmd: Dict[str, Any]) -> bool:
        """Execute navigation using the action mapper"""
        # Get location pose
        location = cmd.get('location', 'unknown')
        pose = self.get_pose_for_location(location)
        
        if not pose:
            self.get_logger().warn(f"Unknown location: {location}")
            return False
        
        # Create navigation parameters
        params = {
            'pose': pose,
            'frame_id': 'map'
        }
        
        # Execute using action mapper
        return self.action_mapper.execute_navigate_to_pose(params)
    
    def execute_mapped_movement(self, cmd: Dict[str, Any]) -> bool:
        """Execute movement using the action mapper"""
        direction = cmd.get('direction', 'forward')
        speed = cmd.get('speed', 0.5)
        
        # Create movement parameters based on direction
        params = {
            'linear_x': 0.0,
            'linear_y': 0.0,
            'angular_z': 0.0
        }
        
        if direction == 'forward':
            params['linear_x'] = speed
        elif direction == 'backward':
            params['linear_x'] = -speed
        elif direction == 'left':
            params['linear_y'] = speed
        elif direction == 'right':
            params['linear_y'] = -speed
        elif direction == 'rotate_left':
            params['angular_z'] = speed
        elif direction == 'rotate_right':
            params['angular_z'] = -speed
        else:
            self.get_logger().warn(f"Unknown direction: {direction}")
            return False
        
        # Execute using action mapper
        return self.action_mapper.publish_cmd_vel(params)
    
    def execute_mapped_manipulation(self, cmd: Dict[str, Any]) -> bool:
        """Execute manipulation using the action mapper"""
        action = cmd.get('manipulation_action', '')
        
        if action in ['grasp', 'pick_up', 'pickup']:
            return self.execute_mapped_grasp(cmd)
        elif action in ['place', 'put_down']:
            return self.execute_mapped_place(cmd)
        else:
            self.get_logger().warn(f"Unknown manipulation action: {action}")
            return False
    
    def execute_mapped_grasp(self, cmd: Dict[str, Any]) -> bool:
        """Execute grasp using the action mapper"""
        obj_name = cmd.get('target_object', 'unknown')
        
        # Get object pose
        obj_pose = self.get_object_pose(obj_name)
        if not obj_pose:
            self.get_logger().warn(f"Could not find object: {obj_name}")
            return False
        
        # Plan grasp trajectory (simplified)
        # In a real system, this would plan approach and grasp motions
        grasp_joints = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5, 0.0]  # Example joint positions
        
        # Create joint command parameters
        params = {
            'joint_names': [f'joint_{i}' for i in range(len(grasp_joints))],
            'joint_positions': grasp_joints
        }
        
        # Execute using action mapper
        return self.action_mapper.publish_joint_commands(params)
    
    def execute_mapped_place(self, cmd: Dict[str, Any]) -> bool:
        """Execute place using the action mapper"""
        # Implement place action
        # This would involve moving to place position and opening gripper
        place_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Open gripper position
        
        # Create joint command parameters
        params = {
            'joint_names': [f'joint_{i}' for i in range(len(place_joints))],
            'joint_positions': place_joints
        }
        
        # Execute using action mapper
        return self.action_mapper.publish_joint_commands(params)
    
    def execute_mapped_transport(self, cmd: Dict[str, Any]) -> bool:
        """Execute transport action (fetch and carry)"""
        target_obj = cmd.get('target_object', 'unknown')
        
        # First, navigate to object
        obj_pose = self.get_object_pose(target_obj)
        if not obj_pose:
            self.get_logger().warn(f"Could not find object: {target_obj}")
            return False
        
        # Grasp the object
        grasp_cmd = {
            'manipulation_action': 'grasp',
            'target_object': target_obj
        }
        
        if not self.execute_mapped_grasp(grasp_cmd):
            return False
        
        # Navigate to destination
        destination = cmd.get('destination', 'unknown')
        if destination != 'unknown':
            nav_cmd = {'location': destination}
            return self.execute_mapped_navigation(nav_cmd)
        
        return True
```

## Handling Ambiguity and Context

### Implementing Context-Aware Command Processing:

```python
class ContextAwareNaturalLanguageToROS2(MappedNaturalLanguageToROS2):
    def __init__(self):
        super().__init__()
        
        # Initialize context
        self.current_context = {
            'last_object': None,
            'last_location': None,
            'robot_position': None,
            'task_history': [],
            'user_preferences': {}
        }
        
        # Subscribe to position updates
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
    
    def odom_callback(self, msg: Odometry):
        """Update robot position in context"""
        self.current_context['robot_position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def command_callback(self, msg):
        """Process command with context awareness"""
        command = msg.data
        self.get_logger().info(f"Processing contextual command: {command}")
        
        # Resolve references using context
        resolved_command = self.resolve_references(command)
        
        # Parse command
        parsed_command = self.enhanced_parser.parse_command(resolved_command)
        
        if not parsed_command:
            self.get_logger().warn(f"Could not parse command: {command}")
            self.publish_feedback(f"Sorry, I didn't understand: {command}")
            return
        
        # Map to ROS 2 action
        success = self.execute_ros2_action(parsed_command)
        
        if success:
            self.get_logger().info(f"Successfully executed command: {command}")
            self.publish_feedback(f"Done: {command}")
            
            # Update context
            self.update_context(command, parsed_command)
        else:
            self.get_logger().error(f"Failed to execute command: {command}")
            self.publish_feedback(f"Failed to execute: {command}")
    
    def resolve_references(self, command: str) -> str:
        """Resolve ambiguous references using context"""
        # Replace pronouns and demonstratives with specific references
        command_lower = command.lower()
        
        # Replace "it" with last mentioned object
        if "it" in command_lower and self.current_context['last_object']:
            command = command.replace(" it ", f" {self.current_context['last_object']} ")
        
        # Replace "there" with last mentioned location
        if "there" in command_lower and self.current_context['last_location']:
            command = command.replace(" there", f" {self.current_context['last_location']}")
        
        # Replace "this" with last mentioned object
        if "this" in command_lower and self.current_context['last_object']:
            command = command.replace(" this", f" {self.current_context['last_object']}")
        
        return command
    
    def update_context(self, command: str, parsed_command: Dict[str, Any]):
        """Update context based on executed command"""
        # Update last object if mentioned
        if 'target_object' in parsed_command:
            self.current_context['last_object'] = parsed_command['target_object']
        
        # Update last location if mentioned
        if 'location' in parsed_command:
            self.current_context['last_location'] = parsed_command['location']
        
        # Update task history
        self.current_context['task_history'].append({
            'command': command,
            'parsed': parsed_command,
            'timestamp': self.get_clock().now().nanoseconds
        })
    
    def query_context(self, entity_type: str) -> Optional[Any]:
        """Query context for specific information"""
        return self.current_context.get(entity_type)
    
    def handle_ambiguous_command(self, command: str) -> List[str]:
        """Handle ambiguous commands requiring clarification"""
        # Detect what's ambiguous in the command
        ambiguity_flags = {
            'object_unclear': self.is_object_unclear(command),
            'location_unclear': self.is_location_unclear(command),
            'action_unclear': self.is_action_unclear(command)
        }
        
        questions = []
        
        if ambiguity_flags['object_unclear']:
            questions.append(f"Which object do you mean in '{command}'?")
        
        if ambiguity_flags['location_unclear']:
            questions.append(f"Where exactly do you want me to go in '{command}'?")
        
        if ambiguity_flags['action_unclear']:
            questions.append(f"What do you want me to do in '{command}'?")
        
        return questions if questions else [f"Can you be more specific about '{command}'?"]
    
    def is_object_unclear(self, command: str) -> bool:
        """Check if command has unclear object reference"""
        unclear_indicators = ['it', 'that', 'thing', 'object', 'the']
        words = command.lower().split()
        
        # Check for vague object references
        for indicator in unclear_indicators:
            if indicator in words:
                # Check if followed by a specific object
                idx = words.index(indicator)
                if idx + 1 < len(words):
                    next_word = words[idx + 1]
                    if next_word in ['one', 'thing', 'stuff', 'here', 'there']:
                        return True
        
        return False
    
    def is_location_unclear(self, command: str) -> bool:
        """Check if command has unclear location reference"""
        unclear_locations = ['there', 'here', 'that place', 'somewhere', 'around']
        return any(loc in command.lower() for loc in unclear_locations)
    
    def is_action_unclear(self, command: str) -> bool:
        """Check if command has unclear action"""
        # If command is very general
        general_actions = ['do', 'go', 'move', 'work', 'something']
        words = command.lower().split()
        return len([w for w in words if w in general_actions]) > 0 and len(words) < 4
```

## Performance Evaluation

### Implementing Evaluation Metrics:

```python
class NaturalLanguageToROS2Metrics:
    def __init__(self):
        self.metrics = {
            'command_accuracy': 0.0,
            'execution_success_rate': 0.0,
            'ambiguity_detection_rate': 0.0,
            'context_utilization_rate': 0.0,
            'average_response_time': 0.0,
            'semantic_parsing_accuracy': 0.0
        }
        
        self.command_attempts = 0
        self.command_successes = 0
        self.ambiguity_detected = 0
        self.ambiguity_correctly_handled = 0
        self.context_used = 0
        self.response_times = []
        self.parsing_correct = 0
        self.parsing_attempts = 0
    
    def record_command_attempt(self, command: str, success: bool):
        """Record command processing attempt"""
        self.command_attempts += 1
        if success:
            self.command_successes += 1
        
        self.metrics['command_accuracy'] = (
            self.command_successes / self.command_attempts 
            if self.command_attempts > 0 else 0
        )
    
    def record_ambiguity_handling(self, command: str, was_ambiguous: bool, handled_correctly: bool):
        """Record ambiguity handling"""
        if was_ambiguous:
            self.ambiguity_detected += 1
            if handled_correctly:
                self.ambiguity_correctly_handled += 1
        
        self.metrics['ambiguity_detection_rate'] = (
            self.ambiguity_correctly_handled / self.ambiguity_detected 
            if self.ambiguity_detected > 0 else 0
        )
    
    def record_context_usage(self, used: bool):
        """Record context usage"""
        if used:
            self.context_used += 1
        
        total_commands = self.command_attempts if self.command_attempts > 0 else 1
        self.metrics['context_utilization_rate'] = self.context_used / total_commands
    
    def record_response_time(self, time_ms: float):
        """Record response time"""
        self.response_times.append(time_ms)
        if self.response_times:
            self.metrics['average_response_time'] = sum(self.response_times) / len(self.response_times)
    
    def record_parsing_accuracy(self, command: str, parsed_correctly: bool):
        """Record semantic parsing accuracy"""
        self.parsing_attempts += 1
        if parsed_correctly:
            self.parsing_correct += 1
        
        self.metrics['semantic_parsing_accuracy'] = (
            self.parsing_correct / self.parsing_attempts 
            if self.parsing_attempts > 0 else 0
        )
    
    def generate_report(self):
        """Generate performance report"""
        report = f"""
        Natural Language to ROS 2 Performance Report:
        - Command Accuracy: {self.metrics['command_accuracy']:.2%}
        - Execution Success Rate: {self.metrics['execution_success_rate']:.2%}
        - Ambiguity Detection Rate: {self.metrics['ambiguity_detection_rate']:.2%}
        - Context Utilization Rate: {self.metrics['context_utilization_rate']:.2%}
        - Average Response Time: {self.metrics['average_response_time']:.2f}ms
        - Semantic Parsing Accuracy: {self.metrics['semantic_parsing_accuracy']:.2%}
        """
        return report


def main(args=None):
    rclpy.init(args=args)
    nl_to_ros2 = ContextAwareNaturalLanguageToROS2()
    
    try:
        rclpy.spin(nl_to_ros2)
    except KeyboardInterrupt:
        pass
    finally:
        nl_to_ros2.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Natural Language to ROS 2

### 1. Command Design:
- Use consistent command patterns
- Implement fallback responses for unknown commands
- Design commands with clear intent
- Provide user feedback on command status

### 2. Context Management:
- Maintain relevant context for reference resolution
- Update context appropriately after actions
- Implement context expiration for outdated information
- Allow users to clear context when needed

### 3. Error Handling:
- Gracefully handle parsing failures
- Implement robust error recovery
- Provide meaningful error messages
- Maintain system state during failures

### 4. Safety Considerations:
- Validate commands before execution
- Implement safety checks for robot actions
- Monitor robot state during execution
- Provide emergency stop capabilities

## Summary

This chapter covered the implementation of natural language to ROS 2 action conversion systems. We explored parsing techniques, action mapping, context management, and ambiguity resolution.

The key components include language understanding, action mapping, parameter extraction, and execution. Context-aware processing helps resolve ambiguities and maintain task continuity. The system must handle various command types including navigation, manipulation, and complex multi-step tasks.

Successful natural language to ROS 2 conversion requires careful design of parsing rules, comprehensive action mappings, and robust error handling to ensure reliable robot operation.

## Exercises

1. Implement a natural language interface for your robot that supports navigation commands.
2. Create a semantic parser for manipulation tasks using your robot's capabilities.
3. Implement context management for reference resolution in multi-sentence commands.
4. Evaluate your system's performance with various natural language command variations.
5. Add safety checks and validation for natural language commands to prevent unsafe robot behavior.