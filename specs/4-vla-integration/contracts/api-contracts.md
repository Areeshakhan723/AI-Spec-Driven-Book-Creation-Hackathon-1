# Module 4: Vision-Language-Action (VLA) - API Contracts

## Overview

This document defines the API contracts for the Vision-Language-Action (VLA) system, including interfaces between voice processing, LLM cognitive planning, navigation, perception, and manipulation components.

## Voice Processing Interfaces

### Voice Command Interface
- **Topic**: `/natural_language_command`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when voice command is processed)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # The transcribed voice command
```

### Voice Command Status Interface
- **Topic**: `/voice_command_status`
- **Type**: `vla_interfaces/VoiceCommand`
- **Frequency**: Event-driven (when command is processed)

**Message Structure**:
```
builtin_interfaces/Time timestamp
string command
float32 confidence
string status  # "received", "processing", "completed", "failed"
```

### Whisper Audio Processing Interface
- **Service**: `/process_audio`
- **Type**: Custom service for audio processing
- **Frequency**: On-demand

**Request**:
```
sensor_msgs/Float32MultiArray audio_data
string model_size  # tiny, base, small, medium, large
float32 timeout
```

**Response**:
```
bool success
string transcribed_text
float32 processing_time
float32 confidence
```

## LLM Cognitive Planning Interfaces

### Action Plan Request Interface
- **Topic**: `/plan_request`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when plan is needed)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # JSON string containing command and context
```

### Generated Action Plan Interface
- **Topic**: `/generated_action_plan`
- **Type**: `vla_interfaces/ActionPlan`
- **Frequency**: Event-driven (when plan is generated)

**Message Structure**:
```
std_msgs/Header header
string command  # Original command
ActionStep[] steps  # Sequence of action steps
float32 estimated_duration  # Estimated time in seconds
float32 confidence  # Confidence level (0-1)
builtin_interfaces/Time timestamp
string[] safety_checks  # List of safety checks to perform
```

### Action Step Interface
- **Component of ActionPlan message**
- **Type**: `vla_interfaces/ActionStep`

**Message Structure**:
```
uint32 step_number
string action_type  # navigation, manipulation, perception, other
string description  # Detailed action description
string parameters  # JSON string of action parameters
string[] preconditions  # List of required preconditions
string expected_outcome  # Expected result of action
float32 timeout  # Timeout in seconds
```

## Navigation Interfaces

### Navigation Goal Interface
- **Action**: `/navigate_to_pose`
- **Type**: `nav2_msgs/NavigateToPose`
- **Frequency**: As needed for navigation

**Goal Message**:
```
geometry_msgs/PoseStamped pose
string behavior_tree  # Behavior tree to use for navigation
```

**Result Message**:
```
uint8 outcome  # 1=success, 2=canceled, 3=failed
builtin_interfaces/Duration execution_time
geometry_msgs/PoseStamped final_pose
```

### Navigation Status Interface
- **Topic**: `/navigation_result`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when navigation completes)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # "success", "failure", "timeout", "canceled"
```

## Perception Interfaces

### Object Detection Request Interface
- **Topic**: `/object_detection_request`
- **Type**: `std_msgs/String`
- **Frequency**: As needed for perception tasks

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # Object name to detect (e.g., "red cup", "book")
```

### Object Detection Result Interface
- **Topic**: `/object_detection_results`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when detection is complete)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # JSON string containing detection results
{
  "object_name": "red cup",
  "bbox": [x, y, width, height],
  "center": [center_x, center_y],
  "relative_position": [rel_x, rel_y],
  "confidence": 0.85,
  "area": 1250.0
}
```

### Camera Image Interface
- **Topic**: `/camera/image_raw`
- **Type**: `sensor_msgs/Image`
- **Frequency**: 30 Hz (typical)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

## Manipulation Interfaces

### Manipulation Command Interface
- **Topic**: `/manipulation/command`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when manipulation is needed)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # Command in format: "action:object" or "action:location"
# Examples: "pick_up:red_cup", "place:kitchen_table", "grasp:book"
```

### Manipulation Status Interface
- **Topic**: `/manipulation/status`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when manipulation status changes)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # Status: "idle", "executing", "completed", "error", "gripper_open", "gripper_closed"
```

## System Integration Interfaces

### System Status Interface
- **Topic**: `/system_status`
- **Type**: `std_msgs/String`
- **Frequency**: 1 Hz (or as needed)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # System status in format: "component:status:details"
```

### Environment Context Interface
- **Topic**: `/environment_context`
- **Type**: `std_msgs/String`
- **Frequency**: As needed (when context changes)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # JSON string containing environment context
{
  "timestamp": 1699123456.789,
  "known_locations": ["kitchen", "living room", "bedroom"],
  "detected_objects": {...},
  "obstacles": [...],
  "map_info": {...}
}
```

## Error Handling

### Standard Error Responses
- **Success**: `bool success = true`
- **Failure**: `bool success = false` with descriptive `status_message`
- **Timeout**: Standard ROS service timeout (typically 3-10 seconds)

### Error Message Interface
- **Topic**: `/execution_error`
- **Type**: `std_msgs/String`
- **Frequency**: Event-driven (when errors occur)

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string data  # Error details in format: "component:step:error_description"
```

## Performance Requirements

### Latency Requirements
- Voice processing: < 1000ms end-to-end
- LLM planning: < 5000ms for complex plans
- Navigation: < 100ms for basic commands
- Perception: < 500ms for object detection
- Manipulation: < 1000ms for basic actions

### Throughput Requirements
- Voice commands: Up to 1 command every 2 seconds
- Action plans: Up to 10 steps per plan
- Object detection: Up to 30 Hz for real-time processing
- Navigation updates: Up to 10 Hz during movement

### Reliability
- 99.5% message delivery for critical control interfaces
- 99% message delivery for status interfaces
- Automatic reconnection for long-running services
- Graceful degradation when components fail