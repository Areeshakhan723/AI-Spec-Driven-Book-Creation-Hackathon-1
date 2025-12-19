---
sidebar_position: 3
title: "Chapter 2: Voice-to-Action (Whisper)"
---

# Chapter 2: Voice-to-Action (Whisper)

This chapter focuses on implementing voice command processing using OpenAI Whisper for robotics applications. You'll learn how to convert spoken commands into text that can be processed by LLMs and converted to robotic actions.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up and configure OpenAI Whisper for voice command processing
- Implement real-time speech-to-text conversion for robotics
- Parse voice commands and convert them to ROS 2 messages
- Handle voice command validation and error recovery

## Introduction to Voice Command Processing

Voice command processing enables natural human-robot interaction by allowing users to speak commands in natural language. The process involves:

1. **Audio Capture**: Recording user voice commands
2. **Speech-to-Text**: Converting speech to text using Whisper
3. **Command Parsing**: Extracting actionable elements from the text
4. **ROS 2 Integration**: Converting commands to ROS 2 messages

## Setting up OpenAI Whisper

### Installation and Dependencies

First, install the required packages for Whisper integration:

```bash
pip3 install openai-whisper
pip3 install pyaudio  # For audio capture
pip3 install sounddevice  # Alternative audio library
pip3 install numpy
```

### Whisper Model Options

Whisper comes in several sizes with different performance characteristics:

- **tiny**: Fastest, least accurate (74M parameters)
- **base**: Good balance (145M parameters)
- **small**: Better accuracy (444M parameters)
- **medium**: High accuracy (769M parameters)
- **large**: Highest accuracy (1550M parameters)

For robotics applications, consider using smaller models for real-time processing or larger models for accuracy in offline processing.

### Basic Whisper Implementation

```python
import whisper
import numpy as np
import pyaudio
import wave
import threading
import queue
import time

class WhisperVoiceProcessor:
    def __init__(self, model_size="base", device="cpu"):
        """
        Initialize Whisper voice processor
        model_size: tiny, base, small, medium, large, or multilingual variants
        device: cpu or cuda
        """
        self.model = whisper.load_model(model_size, device=device)
        self.is_listening = False
        self.audio_queue = queue.Queue()

        # Audio parameters
        self.rate = 16000  # Sample rate
        self.chunk = 1024  # Buffer size
        self.format = pyaudio.paInt16
        self.channels = 1

    def start_listening(self):
        """Start audio capture in a separate thread"""
        self.is_listening = True
        audio_thread = threading.Thread(target=self._capture_audio)
        audio_thread.daemon = True
        audio_thread.start()

    def stop_listening(self):
        """Stop audio capture"""
        self.is_listening = False

    def _capture_audio(self):
        """Capture audio from microphone"""
        p = pyaudio.PyAudio()

        stream = p.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        while self.is_listening:
            data = stream.read(self.chunk)
            audio_data = np.frombuffer(data, dtype=np.int16)
            self.audio_queue.put(audio_data.astype(np.float32) / 32768.0)

        stream.stop_stream()
        stream.close()
        p.terminate()

    def process_audio(self, audio_data):
        """Process audio data with Whisper"""
        # Convert audio data to the right format
        if isinstance(audio_data, list):
            audio_data = np.concatenate(audio_data)

        # Transcribe using Whisper
        result = self.model.transcribe(audio_data)
        return result["text"].strip()
```

## Real-time Voice Processing

### Continuous Voice Command Processing

For real-time robotics applications, we need to process voice commands as they occur:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import VoiceCommand
import threading
import time

class RealTimeVoiceNode(Node):
    def __init__(self):
        super().__init__('real_time_voice_node')

        # Publisher for processed voice commands
        self.command_pub = self.create_publisher(
            String,
            'natural_language_command',
            10
        )

        # Publisher for voice command status
        self.status_pub = self.create_publisher(
            VoiceCommand,
            'voice_command_status',
            10
        )

        # Initialize Whisper processor
        self.whisper_processor = WhisperVoiceProcessor(model_size="base")

        # Voice activity detection parameters
        self.silence_threshold = 0.01
        self.min_speech_duration = 0.5  # seconds
        self.max_silence_duration = 1.0  # seconds

        # Start voice processing
        self.processing_thread = threading.Thread(target=self.process_voice_commands)
        self.processing_thread.daemon = True
        self.processing_thread.start()

        self.get_logger().info("Real-time voice processing node started")

    def process_voice_commands(self):
        """Process voice commands in real-time"""
        audio_buffer = []
        silence_duration = 0
        speech_started = False

        while rclpy.ok():
            try:
                # Get audio chunk from processor
                if not self.whisper_processor.audio_queue.empty():
                    audio_chunk = self.whisper_processor.audio_queue.get()

                    # Calculate audio level
                    audio_level = np.mean(np.abs(audio_chunk))

                    if audio_level > self.silence_threshold:
                        # Speech detected
                        audio_buffer.append(audio_chunk)
                        silence_duration = 0
                        speech_started = True
                    else:
                        # Silence detected
                        silence_duration += len(audio_chunk) / self.whisper_processor.rate

                        if speech_started:
                            audio_buffer.append(audio_chunk)

                        # Check if we have a complete command
                        if (speech_started and
                            (silence_duration > self.max_silence_duration or
                             len(audio_buffer) > self.whisper_processor.rate * 5)):  # 5 seconds max

                            if len(audio_buffer) > self.whisper_processor.rate * self.min_speech_duration:
                                # Process the complete command
                                self.process_complete_command(audio_buffer)

                            # Reset for next command
                            audio_buffer = []
                            speech_started = False

            except Exception as e:
                self.get_logger().error(f"Error in voice processing: {e}")

            time.sleep(0.01)  # Small delay to prevent excessive CPU usage

    def process_complete_command(self, audio_buffer):
        """Process a complete voice command"""
        try:
            # Transcribe the audio buffer
            command_text = self.whisper_processor.process_audio(audio_buffer)

            if command_text and len(command_text.strip()) > 0:
                # Publish the command
                cmd_msg = String()
                cmd_msg.data = command_text
                self.command_pub.publish(cmd_msg)

                # Publish status
                status_msg = VoiceCommand()
                status_msg.command = command_text
                status_msg.timestamp = self.get_clock().now().to_msg()
                status_msg.confidence = 0.9  # Placeholder confidence
                self.status_pub.publish(status_msg)

                self.get_logger().info(f"Voice command received: {command_text}")

        except Exception as e:
            self.get_logger().error(f"Error processing command: {e}")
```

## Command Parsing and Validation

### Voice Command Parser

Once we have the transcribed text, we need to parse it and extract actionable elements:

```python
import re
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class ParsedCommand:
    intent: str  # navigation, manipulation, perception, etc.
    parameters: dict
    confidence: float
    original_text: str

class VoiceCommandParser:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'go to (.+)',
                r'move to (.+)',
                r'go to the (.+)',
                r'navigate to (.+)',
                r'go (.+)',
                r'head to (.+)'
            ],
            'manipulation': [
                r'pick up (.+)',
                r'grasp (.+)',
                r'get (.+)',
                r'take (.+)',
                r'pick (.+)',
                r'lift (.+)'
            ],
            'perception': [
                r'find (.+)',
                r'look for (.+)',
                r'detect (.+)',
                r'search for (.+)',
                r'where is (.+)',
                r'locate (.+)'
            ],
            'action': [
                r'clean (.+)',
                r'organize (.+)',
                r'arrange (.+)',
                r'put (.+) in (.+)',
                r'place (.+) on (.+)'
            ]
        }

        # Location keywords
        self.locations = {
            'kitchen': ['kitchen', 'cooking area', 'cooking room'],
            'living room': ['living room', 'living area', 'lounge'],
            'bedroom': ['bedroom', 'sleeping area', 'bed area'],
            'office': ['office', 'study', 'work area'],
            'dining room': ['dining room', 'dining area', 'eat area'],
            'bathroom': ['bathroom', 'restroom', 'toilet']
        }

        # Object keywords
        self.objects = {
            'cup': ['cup', 'mug', 'glass'],
            'book': ['book', 'novel', 'magazine', 'textbook'],
            'bottle': ['bottle', 'water bottle', 'drink'],
            'box': ['box', 'container', 'package'],
            'chair': ['chair', 'seat', 'stool'],
            'table': ['table', 'desk', 'surface']
        }

    def parse_command(self, text: str) -> Optional[ParsedCommand]:
        """Parse voice command and extract intent and parameters"""
        text = text.lower().strip()

        # Try to match command patterns
        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text)
                if match:
                    parameters = self.extract_parameters(intent, match.groups(), text)
                    return ParsedCommand(
                        intent=intent,
                        parameters=parameters,
                        confidence=0.8,  # Placeholder confidence
                        original_text=text
                    )

        # If no pattern matches, return None or a general command
        return None

    def extract_parameters(self, intent: str, groups: tuple, text: str) -> dict:
        """Extract parameters based on intent and matched groups"""
        params = {}

        if intent == 'navigation':
            # Extract location
            location = groups[0].strip()
            params['target_location'] = self.normalize_location(location)

        elif intent == 'manipulation':
            # Extract object
            obj = groups[0].strip()
            params['target_object'] = self.normalize_object(obj)

        elif intent == 'perception':
            # Extract object to find
            obj = groups[0].strip()
            params['target_object'] = self.normalize_object(obj)

        elif intent == 'action':
            # Handle multi-part commands like "put cup in box"
            if len(groups) >= 2:
                params['action_object'] = self.normalize_object(groups[0].strip())
                params['target_location'] = self.normalize_location(groups[1].strip())

        return params

    def normalize_location(self, location: str) -> str:
        """Normalize location names to standard format"""
        for standard_loc, variants in self.locations.items():
            for variant in variants:
                if variant in location:
                    return standard_loc
        return location  # Return as-is if no match found

    def normalize_object(self, obj: str) -> str:
        """Normalize object names to standard format"""
        for standard_obj, variants in self.objects.items():
            for variant in variants:
                if variant in obj:
                    return standard_obj
        return obj  # Return as-is if no match found
```

## ROS 2 Messaging Integration

### Voice Command to ROS 2 Message Conversion

```python
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from vla_interfaces.msg import ParsedVoiceCommand

class VoiceToROSConverter(Node):
    def __init__(self):
        super().__init__('voice_to_ros_converter')

        # Subscribers
        self.voice_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.voice_callback,
            10
        )

        # Publishers for different action types
        self.nav_pub = self.create_publisher(
            PoseStamped,
            'navigation_goal',
            10
        )

        self.action_pub = self.create_publisher(
            ParsedVoiceCommand,
            'parsed_voice_command',
            10
        )

        # Command parser
        self.parser = VoiceCommandParser()

        self.get_logger().info("Voice to ROS converter started")

    def voice_callback(self, msg):
        """Process incoming voice command"""
        command_text = msg.data

        # Parse the command
        parsed_cmd = self.parser.parse_command(command_text)

        if parsed_cmd:
            # Convert to ROS message
            ros_msg = ParsedVoiceCommand()
            ros_msg.intent = parsed_cmd.intent
            ros_msg.parameters = str(parsed_cmd.parameters)  # Convert dict to string
            ros_msg.confidence = parsed_cmd.confidence
            ros_msg.original_command = parsed_cmd.original_text
            ros_msg.timestamp = self.get_clock().now().to_msg()

            # Publish parsed command
            self.action_pub.publish(ros_msg)

            # Handle specific intents
            if parsed_cmd.intent == 'navigation':
                self.handle_navigation_command(parsed_cmd.parameters)
            elif parsed_cmd.intent == 'manipulation':
                self.handle_manipulation_command(parsed_cmd.parameters)
            # Add other intent handlers as needed
        else:
            self.get_logger().warning(f"Could not parse command: {command_text}")

    def handle_navigation_command(self, params):
        """Handle navigation commands"""
        # Convert location to pose
        pose = self.location_to_pose(params.get('target_location', ''))
        if pose:
            self.nav_pub.publish(pose)

    def location_to_pose(self, location: str) -> Optional[PoseStamped]:
        """Convert location name to PoseStamped"""
        # This would typically use a map of known locations
        location_map = {
            'kitchen': (1.0, 2.0, 0.0),  # (x, y, theta)
            'living room': (0.0, 0.0, 0.0),
            'bedroom': (-1.0, 1.0, 1.57),
            'office': (2.0, -1.0, -1.57)
        }

        if location in location_map:
            x, y, theta = location_map[location]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Convert theta to quaternion
            import math
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy
            return pose

        return None

    def handle_manipulation_command(self, params):
        """Handle manipulation commands"""
        # This would trigger manipulation actions
        # For now, just log the command
        self.get_logger().info(f"Manipulation command: {params}")
```

## Voice Command Validation and Error Handling

### Validation System

```python
class VoiceCommandValidator:
    def __init__(self):
        self.known_locations = set(['kitchen', 'living room', 'bedroom', 'office', 'dining room', 'bathroom'])
        self.known_objects = set(['cup', 'book', 'bottle', 'box', 'chair', 'table'])

    def validate_command(self, parsed_command: ParsedCommand) -> tuple[bool, str]:
        """Validate parsed command for safety and feasibility"""
        if not parsed_command:
            return False, "No command to validate"

        # Check if intent is recognized
        if parsed_command.intent not in ['navigation', 'manipulation', 'perception', 'action']:
            return False, f"Unknown intent: {parsed_command.intent}"

        # Validate parameters based on intent
        if parsed_command.intent == 'navigation':
            target_location = parsed_command.parameters.get('target_location')
            if not target_location:
                return False, "Navigation command missing target location"
            if target_location not in self.known_locations:
                return False, f"Unknown location: {target_location}"

        elif parsed_command.intent == 'manipulation':
            target_object = parsed_command.parameters.get('target_object')
            if not target_object:
                return False, "Manipulation command missing target object"
            if target_object not in self.known_objects:
                return False, f"Unknown object: {target_object}"

        # Additional safety checks can be added here
        return True, "Command is valid"
```

## Practical Exercise: Voice Command System

Set up a complete voice command processing system:

1. Install Whisper and audio dependencies
2. Configure the voice processing node
3. Test with simple navigation commands
4. Validate command parsing accuracy

### Complete System Launch File

```xml
<!-- voice_command_system.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Voice processing node
        Node(
            package='vla_voice_processing',
            executable='real_time_voice_node',
            name='real_time_voice_node',
            parameters=[
                {'model_size': 'base'},
                {'device': 'cpu'}
            ],
            output='screen'
        ),

        # Voice to ROS converter
        Node(
            package='vla_voice_processing',
            executable='voice_to_ros_converter',
            name='voice_to_ros_converter',
            output='screen'
        ),

        # LLM interface (from Chapter 1)
        Node(
            package='vla_llm_interface',
            executable='llm_interface_node',
            name='llm_interface_node',
            output='screen'
        )
    ])
```

## Performance Optimization

### Optimizing Whisper for Real-time Processing

For robotics applications, you may need to optimize Whisper performance:

1. **Model Selection**: Use smaller models for faster processing
2. **Audio Chunking**: Process audio in smaller chunks for lower latency
3. **Hardware Acceleration**: Use GPU if available
4. **Caching**: Cache common command transcriptions

```python
class OptimizedWhisperProcessor(WhisperVoiceProcessor):
    def __init__(self, model_size="tiny", device="cpu"):
        super().__init__(model_size, device)

        # Cache for common commands
        self.command_cache = {}

    def process_audio(self, audio_data):
        """Process audio with caching for common commands"""
        # Create a simple hash of the audio data for caching
        audio_hash = hash(tuple(audio_data[:100]))  # First 100 samples as hash

        if audio_hash in self.command_cache:
            return self.command_cache[audio_hash]

        result = super().process_audio(audio_data)

        # Cache the result (be mindful of memory usage)
        if len(self.command_cache) < 1000:  # Limit cache size
            self.command_cache[audio_hash] = result

        return result
```

## Summary

In this chapter, you learned how to implement voice command processing using OpenAI Whisper for robotics applications. You explored real-time audio capture, speech-to-text conversion, command parsing, and integration with ROS 2 messaging. You also learned about validation and error handling for voice commands. In the next chapter, we'll explore cognitive planning using LLMs to convert natural language commands into executable action plans.