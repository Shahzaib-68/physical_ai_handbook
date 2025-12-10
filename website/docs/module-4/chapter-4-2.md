---
sidebar_position: 17
slug: /module-4/chapter-4-2
title: Chapter 4.2 - Voice-to-Action with OpenAI Whisper
---

# Chapter 4.2: Voice-to-Action with OpenAI Whisper

## Overview

Voice interfaces enable natural human-robot interaction, allowing users to control robots using spoken commands. OpenAI Whisper provides state-of-the-art automatic speech recognition (ASR) capabilities that can be integrated with robotic systems to create sophisticated voice-controlled robots. This chapter explores the implementation of voice-to-action systems using Whisper for robotics applications.

## Learning Objectives

By the end of this chapter, students will be able to:
- Understand the principles of automatic speech recognition for robotics
- Implement Whisper-based speech-to-text processing in ROS 2
- Design voice command grammars and parsing systems
- Integrate voice recognition with robot control systems
- Optimize Whisper performance for real-time robotic applications
- Evaluate voice interface performance and robustness

## Introduction to Voice Interfaces for Robotics

Voice interfaces provide an intuitive way for humans to interact with robots, eliminating the need for specialized programming knowledge or physical interfaces. For robotics applications, voice technology involves:

### Key Components:
1. **Speech Recognition**: Converting spoken words to text
2. **Natural Language Understanding**: Interpreting human intent
3. **Command Mapping**: Converting understanding to robot actions
4. **Feedback Mechanisms**: Confirming actions or requesting clarifications

### Benefits of Voice Control:
- Natural, intuitive interaction
- Hands-free operation
- Accessibility for users with mobility limitations
- Efficient for complex task specification

### Challenges in Robotics Context:
- Noisy environments affecting recognition quality
- Real-time processing requirements
- Domain-specific vocabulary and commands
- Handling of ambiguous or incomplete commands
- Safety considerations for autonomous actions

## OpenAI Whisper for Robotics

### Whisper Architecture:
Whisper is a robust speech recognition model trained on diverse audio data. Key features include:
- Multilingual support with strong performance across languages
- Robustness to background noise and audio quality variations
- End-to-end architecture requiring minimal preprocessing
- Available in multiple model sizes for different performance needs

### Whisper Model Variants:
- **tiny**: Fastest, lowest accuracy
- **base**: Good balance of speed and accuracy
- **small**: Better accuracy, moderate speed
- **medium**: High accuracy, slower processing
- **large**: Highest accuracy, slowest processing

## Setting Up Whisper for Robotics

### Installation:
```bash
pip install openai-whisper
# For GPU acceleration (optional but recommended):
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### Basic Whisper Node Implementation:

```python
import rclpy
from rclpy.node import Node
import whisper
import numpy as np
import pyaudio
import wave
import threading
import queue
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import torch

class WhisperVoiceInterface(Node):
    def __init__(self):
        super().__init__('whisper_voice_interface')
        
        # Load Whisper model
        model_size = self.declare_parameter('model_size', 'base').value
        self.get_logger().info(f'Loading Whisper model: {model_size}')
        
        # Load model with GPU support if available
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = whisper.load_model(model_size).to(self.device)
        
        # Audio parameters
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_size = 1024
        self.audio_queue = queue.Queue()
        
        # Publishers and subscribers
        self.transcript_publisher = self.create_publisher(String, '/voice_transcript', 10)
        self.command_publisher = self.create_publisher(String, '/robot_command', 10)
        self.audio_subscriber = self.create_subscription(
            AudioData, '/audio_input', self.audio_callback, 10)
        
        # Voice activity detection
        self.listening = False
        self.recording = False
        self.audio_buffer = []
        
        # Start audio processing thread
        self.processing_thread = threading.Thread(target=self.process_audio, daemon=True)
        self.processing_thread.start()
        
        # Timer for processing audio
        self.process_timer = self.create_timer(0.1, self.check_audio_buffer)
        
        self.get_logger().info(f"Whisper Voice Interface initialized on {self.device}")

    def audio_callback(self, audio_data):
        """Handle incoming audio data"""
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(audio_data.data, dtype=np.int16).astype(np.float32) / 32768.0
            self.audio_queue.put(audio_array)
        except Exception as e:
            self.get_logger().error(f"Error processing audio data: {e}")

    def check_audio_buffer(self):
        """Check for audio data to process"""
        if not self.audio_queue.empty():
            while not self.audio_queue.empty():
                audio_chunk = self.audio_queue.get()
                self.audio_buffer.extend(audio_chunk)
        
        # If we have enough audio, start processing
        if len(self.audio_buffer) > self.sample_rate * 2:  # 2 seconds of audio
            self.process_speech()

    def process_speech(self):
        """Process accumulated audio for speech recognition"""
        if len(self.audio_buffer) < self.sample_rate * 0.5:  # At least 0.5 seconds
            return
        
        # Convert to numpy array and convert to required format for Whisper
        audio_np = np.array(self.audio_buffer, dtype=np.float32)
        
        # Pad or trim to appropriate length
        target_length = self.sample_rate * 10  # 10 seconds max (Whisper limit)
        if len(audio_np) > target_length:
            audio_np = audio_np[:target_length]
        elif len(audio_np) < self.sample_rate:  # Minimum 1 second
            padding = np.zeros(target_length - len(audio_np), dtype=np.float32)
            audio_np = np.concatenate([audio_np, padding])
        
        try:
            # Run Whisper transcription
            result = self.model.transcribe(
                audio_np, 
                language="en",
                fp16=(self.device == "cuda")
            )
            
            transcript = result["text"].strip()
            if transcript:  # Only publish if there's text
                self.get_logger().info(f"Transcript: {transcript}")
                
                # Publish transcript
                transcript_msg = String()
                transcript_msg.data = transcript
                self.transcript_publisher.publish(transcript_msg)
                
                # Process for robot commands
                self.process_command(transcript)
        
        except Exception as e:
            self.get_logger().error(f"Error in speech recognition: {e}")
        
        # Clear buffer after processing
        self.audio_buffer = []

    def process_command(self, transcript):
        """Process transcript for robot commands"""
        # Simple command parsing for demonstration
        if "move forward" in transcript.lower():
            cmd = "MOVE_FORWARD"
        elif "turn left" in transcript.lower():
            cmd = "TURN_LEFT"
        elif "turn right" in transcript.lower():
            cmd = "TURN_RIGHT"
        elif "stop" in transcript.lower():
            cmd = "STOP"
        elif "pick up" in transcript.lower() or "grasp" in transcript.lower():
            cmd = "GRASP_OBJECT"
        elif "bring me" in transcript.lower():
            cmd = "FETCH_ITEM"
        else:
            # Use more sophisticated NLP for complex commands
            cmd = self.parse_complex_command(transcript)
        
        # Publish command
        cmd_msg = String()
        cmd_msg.data = cmd
        self.command_publisher.publish(cmd_msg)
        self.get_logger().info(f"Published command: {cmd}")

    def parse_complex_command(self, transcript):
        """Parse complex commands using NLP techniques"""
        # This is a simplified example - in practice, this would use more sophisticated NLP
        import re
        
        # Look for object, location, action patterns
        patterns = {
            r"bring me the (\w+)": "BRING_OBJECT",
            r"go to the (\w+)": "NAVIGATE_TO_LOCATION",
            r"pick up the (\w+)": "GRAB_OBJECT",
            r"move to (\w+)": "MOVE_TO_LOCATION",
        }
        
        for pattern, command in patterns.items():
            match = re.search(pattern, transcript.lower())
            if match:
                target = match.group(1)
                return f"{command}:{target}"
        
        # If no pattern matches, return as general command
        return f"GENERAL_COMMAND:{transcript}"

def main(args=None):
    rclpy.init(args=args)
    voice_interface = WhisperVoiceInterface()
    
    try:
        rclpy.spin(voice_interface)
    except KeyboardInterrupt:
        pass
    finally:
        voice_interface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Voice Command Processing

### Implementing Command Grammars:

```python
import re
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class RobotCommand:
    action: str
    target: Optional[str] = None
    location: Optional[str] = None
    parameters: dict = None

class VoiceCommandGrammar:
    def __init__(self):
        # Define command patterns
        self.patterns = [
            # Navigation commands
            {
                "pattern": r"move (forward|backward|left|right)",
                "action": "NAVIGATE_DIRECTION",
                "target_group": 1
            },
            {
                "pattern": r"go to the (kitchen|bedroom|living room|office)",
                "action": "NAVIGATE_TO_ROOM",
                "target_group": 1
            },
            {
                "pattern": r"navigate to (.+)",
                "action": "NAVIGATE_TO_LOCATION",
                "target_group": 1
            },
            # Manipulation commands
            {
                "pattern": r"pick up the (\w+)",
                "action": "GRAB_OBJECT",
                "target_group": 1
            },
            {
                "pattern": r"grasp the (\w+)",
                "action": "GRAB_OBJECT",
                "target_group": 1
            },
            {
                "pattern": r"bring me the (\w+)",
                "action": "FETCH_OBJECT",
                "target_group": 1
            },
            # Action commands
            {
                "pattern": r"stop",
                "action": "STOP_ROBOT"
            },
            {
                "pattern": r"turn (left|right)",
                "action": "ROTATE",
                "target_group": 1
            },
            {
                "pattern": r"say hello|greet",
                "action": "SPEAK",
                "parameters": {"text": "Hello! How can I help you?"}
            }
        ]
    
    def parse_command(self, transcript: str) -> Optional[RobotCommand]:
        """Parse natural language command into structured format"""
        transcript = transcript.strip().lower()
        
        for pattern_config in self.patterns:
            match = re.search(pattern_config["pattern"], transcript)
            if match:
                command = RobotCommand(
                    action=pattern_config["action"],
                    target=match.group(pattern_config.get("target_group", 0)) if "target_group" in pattern_config else None,
                    parameters=pattern_config.get("parameters", {})
                )
                return command
        
        return None  # No pattern matched

class AdvancedWhisperInterface(WhisperVoiceInterface):
    def __init__(self):
        super().__init__()
        self.command_grammar = VoiceCommandGrammar()
        
        # Add command history for context
        self.command_history = []
        
        # Add confidence threshold
        self.confidence_threshold = 0.7
    
    def process_command(self, transcript):
        """Process transcript using advanced command parsing"""
        # Parse command using grammar
        parsed_command = self.command_grammar.parse_command(transcript)
        
        if parsed_command:
            # Add to command history
            self.command_history.append({
                "transcript": transcript,
                "command": parsed_command,
                "timestamp": self.get_clock().now()
            })
            
            # Convert to ROS message
            cmd_msg = String()
            cmd_msg.data = f"{parsed_command.action}:{parsed_command.target or ''}"
            self.command_publisher.publish(cmd_msg)
            
            self.get_logger().info(f"Executed command: {parsed_command.action} - {parsed_command.target}")
        else:
            # Command not recognized, publish for higher-level processing
            cmd_msg = String()
            cmd_msg.data = f"UNRECOGNIZED:{transcript}"
            self.command_publisher.publish(cmd_msg)
            self.get_logger().warn(f"Unrecognized command: {transcript}")
```

## Voice Activity Detection and Optimization

### Implementing Voice Activity Detection:

```python
import webrtcvad
import collections

class VoiceActivityDetector:
    def __init__(self, sample_rate=16000, frame_duration_ms=30):
        self.vad = webrtcvad.Vad(2)  # Aggressiveness mode 2
        self.sample_rate = sample_rate
        self.frame_duration_ms = frame_duration_ms
        self.frame_size = int(sample_rate * frame_duration_ms / 1000)
        
        # Keep track of voice activity
        self.ring_buffer = collections.deque(maxlen=30)
        self.triggered = False
        self.num_voiced = 0
        self.num_unvoiced = 0

    def is_speech(self, audio_chunk):
        """Detect if audio chunk contains speech"""
        # Convert float32 to int16 for VAD
        audio_int16 = (audio_chunk * 32767).astype(np.int16)
        
        # Process in frames
        frames = self._split_audio_into_frames(audio_int16)
        voiced_frames = 0
        
        for frame in frames:
            if len(frame) < self.frame_size:
                # Pad frame if too short
                frame = np.pad(frame, (0, self.frame_size - len(frame)), 'constant')
            
            # Convert to bytes for WebRTC VAD
            frame_bytes = frame.tobytes()
            if self.vad.is_speech(frame_bytes, self.sample_rate):
                voiced_frames += 1
        
        # Consider speech if majority of frames are voiced
        return voiced_frames > len(frames) // 2

    def _split_audio_into_frames(self, audio):
        """Split audio into frames for VAD"""
        frames = []
        for i in range(0, len(audio), self.frame_size):
            frame = audio[i:i + self.frame_size]
            if len(frame) == self.frame_size:
                frames.append(frame)
        return frames

class OptimizedWhisperInterface(AdvancedWhisperInterface):
    def __init__(self):
        super().__init__()
        
        # Initialize voice activity detection
        self.vad = VoiceActivityDetector(sample_rate=self.sample_rate)
        
        # Add speech detection parameters
        self.min_speech_duration = 0.5  # Minimum speech duration in seconds
        self.post_speech_silence = 0.5  # Wait time after speech stops
        
        # Audio state management
        self.in_speech = False
        self.speech_start_time = None
        self.silence_start_time = None
        
        # Add feedback publisher
        self.feedback_publisher = self.create_publisher(String, '/voice_feedback', 10)

    def audio_callback(self, audio_data):
        """Enhanced audio callback with VAD"""
        try:
            # Convert audio data to numpy array
            audio_array = np.frombuffer(audio_data.data, dtype=np.int16).astype(np.float32) / 32768.0
            
            # Check for voice activity
            is_speech = self.vad.is_speech(audio_array)
            
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            if is_speech:
                if not self.in_speech:
                    # Start of speech
                    self.in_speech = True
                    self.speech_start_time = current_time
                    self.audio_buffer = []  # Start fresh buffer
                    self.get_logger().info("Speech detected, starting recording")
                    
                    # Provide feedback to user
                    feedback_msg = String()
                    feedback_msg.data = "Listening..."
                    self.feedback_publisher.publish(feedback_msg)
                
                # Add to buffer
                self.audio_buffer.extend(audio_array)
                self.silence_start_time = None  # Reset silence timer
            
            elif self.in_speech:
                # We were in speech but now detected silence
                if self.silence_start_time is None:
                    self.silence_start_time = current_time
                
                # Add audio to buffer even during silence (for natural sentence endings)
                self.audio_buffer.extend(audio_array)
                
                # If sufficient silence after speech, process
                if (current_time - self.silence_start_time) > self.post_speech_silence:
                    self.get_logger().info("End of speech detected, processing...")
                    self.process_speech()
                    self.in_speech = False
                    self.silence_start_time = None
                    
                    # Provide feedback
                    feedback_msg = String()
                    feedback_msg.data = "Processing command..."
                    self.feedback_publisher.publish(feedback_msg)
            
            # If speech is too short to be valid, reset
            elif self.in_speech:
                if (current_time - self.speech_start_time) > (self.min_speech_duration * 1.5):
                    # If we've been recording for too long without silence, process anyway
                    self.process_speech()
                    self.in_speech = False
                    self.silence_start_time = None
        
        except Exception as e:
            self.get_logger().error(f"Error in audio processing: {e}")
```

## Real-time Performance Optimization

### Optimized Whisper Processing:

```python
class RealTimeOptimizedWhisper(OptimizedWhisperInterface):
    def __init__(self):
        super().__init__()
        
        # Use a smaller model for real-time processing
        self.fast_model = whisper.load_model("base").to(self.device)
        
        # Pre-allocate audio buffer
        self.max_buffer_size = self.sample_rate * 5  # 5 seconds max
        
        # Caching mechanism
        self.transcript_cache = {}
        self.cache_size_limit = 100
        
        # Performance monitoring
        self.processing_times = []
    
    def process_speech(self):
        """Optimized speech processing for real-time performance"""
        if len(self.audio_buffer) < self.sample_rate * 0.5:  # At least 0.5 seconds
            return
        
        # Trim buffer to maximum size to avoid long processing times
        if len(self.audio_buffer) > self.max_buffer_size:
            self.audio_buffer = self.audio_buffer[-self.max_buffer_size:]
        
        # Convert to numpy array
        audio_np = np.array(self.audio_buffer, dtype=np.float32)
        
        # Check if this audio is similar to recent processing (for caching)
        audio_hash = hash(audio_np.tobytes())
        
        if audio_hash in self.transcript_cache:
            result = self.transcript_cache[audio_hash]
            self.get_logger().info(f"Using cached transcript: {result}")
        else:
            try:
                import time
                start_time = time.time()
                
                # Use the appropriate model based on requirements
                model_to_use = self.model if len(audio_np) > self.sample_rate * 3 else self.fast_model
                
                result = model_to_use.transcribe(
                    audio_np,
                    language="en",
                    fp16=(self.device == "cuda"),
                    temperature=0  # For consistent results
                )
                
                processing_time = time.time() - start_time
                self.processing_times.append(processing_time)
                
                # Add to cache
                if len(self.transcript_cache) < self.cache_size_limit:
                    self.transcript_cache[audio_hash] = result["text"]
                
                self.get_logger().info(f"Transcription took {processing_time:.2f}s")
                
                # Log performance statistics periodically
                if len(self.processing_times) % 10 == 0:
                    avg_time = sum(self.processing_times) / len(self.processing_times)
                    self.get_logger().info(f"Average processing time: {avg_time:.2f}s")
                
            except Exception as e:
                self.get_logger().error(f"Error in speech recognition: {e}")
                return
        
        transcript = result["text"].strip()
        if transcript:  # Only publish if there's text
            self.get_logger().info(f"Transcript: {transcript}")
            
            # Publish transcript
            transcript_msg = String()
            transcript_msg.data = transcript
            self.transcript_publisher.publish(transcript_msg)
            
            # Process for robot commands
            self.process_command(transcript)
        
        # Clear buffer after processing
        self.audio_buffer = []
```

## Integration with Robot Control Systems

### Voice-to-Action Bridge:

```python
class VoiceControlBridge(Node):
    def __init__(self):
        super().__init__('voice_control_bridge')
        
        # Subscribe to voice commands
        self.voice_command_sub = self.create_subscription(
            String, '/robot_command', self.voice_command_callback, 10)
        
        # Robot control publishers
        self.nav_goal_pub = self.create_publisher(Pose, '/navigation_goal', 10)
        self.arm_command_pub = self.create_publisher(JointState, '/arm_controller/command', 10)
        self.speak_command_pub = self.create_publisher(String, '/speak_command', 10)
        
        # Service clients for robot actions
        self.nav_client = self.create_client(NavigateToPose, '/navigate_to_pose')
        
        self.get_logger().info("Voice Control Bridge Initialized")

    def voice_command_callback(self, msg):
        """Process voice commands and convert to robot actions"""
        command = msg.data
        self.get_logger().info(f"Processing voice command: {command}")
        
        # Parse command into action and parameters
        parts = command.split(':', 1)
        action = parts[0]
        params = parts[1] if len(parts) > 1 else ""
        
        if action == "NAVIGATE_DIRECTION":
            self.execute_navigation_direction(params)
        elif action == "NAVIGATE_TO_ROOM":
            self.execute_navigate_to_room(params)
        elif action == "GRAB_OBJECT":
            self.execute_grab_object(params)
        elif action == "FETCH_OBJECT":
            self.execute_fetch_object(params)
        elif action == "STOP_ROBOT":
            self.execute_stop_robot()
        elif action == "SPEAK":
            self.execute_speak(params)
        elif action.startswith("GENERAL_COMMAND"):
            self.handle_general_command(command)
        else:
            self.get_logger().warn(f"Unknown command action: {action}")

    def execute_navigation_direction(self, direction):
        """Execute navigation in a specific direction"""
        # Get current robot position
        current_pose = self.get_current_robot_pose()
        
        if not current_pose:
            self.get_logger().error("Cannot get current robot pose")
            return
        
        # Calculate new pose based on direction
        new_pose = Pose()
        step_size = 0.5  # meters
        
        if "forward" in direction:
            new_pose.position.x = current_pose.position.x + step_size
            new_pose.position.y = current_pose.position.y
        elif "backward" in direction:
            new_pose.position.x = current_pose.position.x - step_size
            new_pose.position.y = current_pose.position.y
        elif "left" in direction:
            new_pose.position.x = current_pose.position.x
            new_pose.position.y = current_pose.position.y + step_size
        elif "right" in direction:
            new_pose.position.x = current_pose.position.x
            new_pose.position.y = current_pose.position.y - step_size
        else:
            self.get_logger().warn(f"Unknown direction: {direction}")
            return
        
        # Set orientation
        new_pose.orientation = current_pose.orientation
        
        # Publish navigation goal
        self.nav_goal_pub.publish(new_pose)

    def execute_navigate_to_room(self, room_name):
        """Navigate to a specific room"""
        # In a real implementation, this would use a map of room locations
        room_locations = {
            "kitchen": (2.0, 1.0, 0.0),
            "bedroom": (3.0, -2.0, 1.57),
            "living room": (-1.0, 0.0, 0.0),
            "office": (0.5, 2.5, -1.57)
        }
        
        if room_name in room_locations:
            x, y, theta = room_locations[room_name]
            goal_msg = Pose()
            goal_msg.position.x = x
            goal_msg.position.y = y
            goal_msg.position.z = 0.0
            goal_msg.orientation.z = np.sin(theta / 2.0)
            goal_msg.orientation.w = np.cos(theta / 2.0)
            
            self.nav_goal_pub.publish(goal_msg)
            self.get_logger().info(f"Navigating to {room_name} at ({x}, {y})")
        else:
            self.get_logger().warn(f"Unknown room: {room_name}")

    def execute_grab_object(self, object_name):
        """Execute object grasping"""
        self.get_logger().info(f"Attempting to grasp: {object_name}")
        # In a real implementation, this would trigger perception and manipulation
        # systems to identify and grasp the specified object
        
        # Example: Send command to manipulation system
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        joint_state.position = [0.0, 0.5, 0.0, 0.5, 0.0, 0.5]  # Example positions
        self.arm_command_pub.publish(joint_state)

    def execute_fetch_object(self, object_name):
        """Execute fetch object behavior"""
        self.get_logger().info(f"Fetching: {object_name}")
        
        # This would typically be a complex behavior involving:
        # 1. Navigate to object location
        # 2. Identify and approach object
        # 3. Grasp object
        # 4. Navigate back to user
        # 5. Present object to user
        
        # For this example, just attempt to grasp
        self.execute_grab_object(object_name)

    def execute_stop_robot(self):
        """Stop robot motion"""
        self.get_logger().info("Stopping robot")
        # Publish zero velocity commands to all movement systems
        # Implementation would depend on specific robot control interface

    def execute_speak(self, text):
        """Make robot speak the provided text"""
        speak_msg = String()
        speak_msg.data = text
        self.speak_command_pub.publish(speak_msg)

    def handle_general_command(self, command):
        """Handle commands that don't match specific patterns"""
        self.get_logger().info(f"Handling general command: {command}")
        # In a real implementation, this might use an LLM to interpret complex commands

    def get_current_robot_pose(self):
        """Get current robot pose from localization system"""
        # Implementation would depend on specific localization system
        return None  # Placeholder
```

## Performance Evaluation and Tuning

### Voice Interface Metrics:

```python
class VoiceInterfaceMetrics:
    def __init__(self):
        self.metrics = {
            'recognition_accuracy': 0.0,
            'command_success_rate': 0.0,
            'average_response_time': 0.0,
            'false_trigger_rate': 0.0,
            'processing_efficiency': 0.0,
            'user_satisfaction': 0.0
        }
        
        self.recognition_attempts = 0
        self.recognition_successes = 0
        self.command_attempts = 0
        self.command_successes = 0
        self.response_times = []
        self.false_triggers = 0
        self.total_triggers = 0

    def record_recognition_attempt(self, expected_text, actual_text):
        """Record speech recognition attempt"""
        self.recognition_attempts += 1
        if expected_text.lower() in actual_text.lower():
            self.recognition_successes += 1
        
        # Update metrics
        self.metrics['recognition_accuracy'] = (
            self.recognition_successes / self.recognition_attempts 
            if self.recognition_attempts > 0 else 0
        )

    def record_command_attempt(self, success):
        """Record command execution attempt"""
        self.command_attempts += 1
        if success:
            self.command_successes += 1
        
        self.metrics['command_success_rate'] = (
            self.command_successes / self.command_attempts 
            if self.command_attempts > 0 else 0
        )

    def record_response_time(self, time_seconds):
        """Record response time"""
        self.response_times.append(time_seconds)
        if self.response_times:
            self.metrics['average_response_time'] = sum(self.response_times) / len(self.response_times)

    def record_false_trigger(self):
        """Record false trigger event"""
        self.false_triggers += 1
        self.total_triggers += 1
        self.metrics['false_trigger_rate'] = (
            self.false_triggers / self.total_triggers 
            if self.total_triggers > 0 else 0
        )

    def record_true_trigger(self):
        """Record true trigger event"""
        self.total_triggers += 1
        # False trigger rate only considers triggers that are false

    def generate_report(self):
        """Generate performance report"""
        report = f"""
        Voice Interface Performance Report:
        - Recognition Accuracy: {self.metrics['recognition_accuracy']:.2%}
        - Command Success Rate: {self.metrics['command_success_rate']:.2%}
        - Avg Response Time: {self.metrics['average_response_time']:.2f}s
        - False Trigger Rate: {self.metrics['false_trigger_rate']:.2%}
        - Processing Efficiency: {self.metrics['processing_efficiency']:.2%}
        - User Satisfaction: {self.metrics['user_satisfaction']:.2%}
        """
        return report
```

## Best Practices for Voice-Enabled Robotics

### 1. Audio Quality:
- Use high-quality microphones positioned appropriately
- Implement noise reduction and echo cancellation
- Consider environment-specific audio processing
- Test performance in various noise conditions

### 2. Real-time Performance:
- Choose appropriate model size for real-time requirements
- Implement audio buffering and streaming efficiently
- Monitor and optimize processing pipeline
- Use edge computing when possible

### 3. User Experience:
- Provide clear feedback on listening state
- Confirm understood commands before execution
- Implement graceful handling of unrecognized commands
- Allow for command corrections and clarifications

### 4. Safety Considerations:
- Implement safety checks before executing commands
- Use confirmation for potentially unsafe actions
- Monitor for ambiguous or conflicting commands
- Maintain manual override capabilities

## Summary

This chapter covered the implementation of voice-to-action systems using OpenAI Whisper for robotics applications. We explored how to integrate Whisper with ROS 2, implement voice activity detection, and create robust voice command parsing systems.

Key components include real-time audio processing, speech recognition using Whisper, command parsing grammars, and integration with robot control systems. Performance optimization techniques such as model selection, caching, and buffering ensure acceptable response times for interactive applications.

Voice interfaces provide an intuitive way for humans to interact with robots, but require careful attention to audio quality, real-time performance, and safety considerations for effective deployment.

## Exercises

1. Implement a Whisper-based voice interface for a simulated robot in Gazebo.
2. Create a command grammar system for your specific robot platform.
3. Add voice activity detection to reduce unnecessary processing.
4. Evaluate the performance of your voice interface in different acoustic conditions.
5. Implement safety checks and user confirmation for voice commands.