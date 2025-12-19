# Module 4: Vision-Language-Action (VLA) - Data Model

## Overview

This document defines the data models used in the Vision-Language-Action (VLA) system, including voice processing, LLM cognitive planning, navigation, perception, and manipulation components.

## Voice Processing Data Models

### Voice Command Model
```
VoiceCommand {
  id: string (unique identifier)
  timestamp: timestamp (when command was received)
  original_audio: string (path to original audio file, if stored)
  transcribed_text: string (transcribed command text)
  confidence: float (confidence level of transcription, 0-1)
  intent: string (parsed intent: navigation, manipulation, perception, etc.)
  parameters: object (parsed command parameters)
  status: string (received, processing, completed, failed)
  processing_time: float (time taken to process command in seconds)
}
```

### Audio Processing Model
```
AudioProcessingResult {
  id: string (unique identifier)
  input_audio: string (reference to input audio)
  model_size: string (Whisper model size used)
  transcribed_text: string (result of speech-to-text)
  processing_time: float (time taken for processing)
  confidence: float (confidence of transcription)
  language: string (detected language)
  duration: float (duration of audio in seconds)
  sample_rate: int (audio sample rate)
}
```

## LLM Cognitive Planning Data Models

### Action Plan Model
```
ActionPlan {
  id: string (unique identifier)
  command: string (original natural language command)
  steps: ActionStep[] (sequence of action steps)
  estimated_duration: float (estimated total execution time in seconds)
  confidence: float (confidence level of plan, 0-1)
  timestamp: timestamp (when plan was generated)
  safety_checks: string[] (list of safety checks to perform)
  preconditions: string[] (list of required preconditions)
  expected_outcomes: string[] (list of expected outcomes)
  robot_state_context: object (robot state at plan generation time)
  environment_context: object (environment context at plan generation time)
}
```

### Action Step Model
```
ActionStep {
  id: string (unique identifier)
  plan_id: string (reference to parent action plan)
  step_number: int (position in the sequence)
  action_type: string (navigation, manipulation, perception, other)
  description: string (detailed description of the action)
  parameters: object (action-specific parameters)
  preconditions: string[] (conditions that must be true before execution)
  expected_outcome: string (expected result of the action)
  timeout: float (maximum time allowed for execution in seconds)
  priority: int (execution priority, higher = more important)
  dependencies: string[] (IDs of steps this step depends on)
  status: string (pending, executing, completed, failed)
  execution_time: float (actual time taken to execute)
}
```

### LLM Context Model
```
LLMContext {
  id: string (unique identifier)
  robot_capabilities: object (robot's current capabilities)
  environment_state: object (current environment information)
  robot_state: object (current robot state including position, battery, etc.)
  previous_interactions: Interaction[] (history of previous interactions)
  current_task: string (current task being executed)
  safety_constraints: object (safety constraints and limitations)
  timestamp: timestamp (when context was created)
}
```

### Interaction Model
```
Interaction {
  id: string (unique identifier)
  command: string (user command)
  response: string (robot's response or action)
  timestamp: timestamp (when interaction occurred)
  success: boolean (whether interaction was successful)
  feedback: string (user feedback, if any)
}
```

## Navigation Data Models

### Navigation Goal Model
```
NavigationGoal {
  id: string (unique identifier)
  target_pose: Pose (target location and orientation)
  behavior_tree: string (behavior tree to use for navigation)
  timeout: float (maximum time allowed for navigation)
  path_constraints: PathConstraints (path planning constraints)
  execution_status: string (pending, executing, completed, failed)
  start_time: timestamp (when navigation started)
  completion_time: timestamp (when navigation completed)
  actual_path: Pose[] (actual path taken by robot)
  distance_traveled: float (actual distance traveled)
}
```

### Pose Model
```
Pose {
  position: Vector3 (x, y, z coordinates)
  orientation: Quaternion (orientation as quaternion)
  frame_id: string (coordinate frame)
}
```

### Path Constraints Model
```
PathConstraints {
  max_velocity: float (maximum allowed velocity)
  min_turn_radius: float (minimum turn radius)
  obstacle_clearance: float (minimum distance to obstacles)
  preferred_path: Pose[] (preferred path to follow)
  restricted_areas: Polygon[] (areas to avoid)
}
```

## Perception Data Models

### Detected Object Model
```
DetectedObject {
  id: string (unique identifier)
  name: string (object name or class)
  confidence: float (detection confidence, 0-1)
  bounding_box: BoundingBox (2D bounding box in image coordinates)
  center_3d: Vector3 (3D center position in world coordinates)
  dimensions: Vector3 (estimated width, height, depth)
  color: string (dominant color)
  category: string (object category)
  timestamp: timestamp (when object was detected)
  camera_frame: string (camera frame where object was detected)
  relative_position: Vector2 (position relative to robot)
}
```

### Bounding Box Model
```
BoundingBox {
  x: float (top-left x coordinate)
  y: float (top-left y coordinate)
  width: float (width of bounding box)
  height: float (height of bounding box)
  confidence: float (confidence of detection)
}
```

### Perception Result Model
```
PerceptionResult {
  id: string (unique identifier)
  image_reference: string (reference to the image processed)
  detected_objects: DetectedObject[] (list of detected objects)
  timestamp: timestamp (when perception was performed)
  processing_time: float (time taken for perception)
  confidence_threshold: float (confidence threshold used)
  detection_method: string (method used for detection)
  environment_context: object (environment context during detection)
}
```

## Manipulation Data Models

### Manipulation Task Model
```
ManipulationTask {
  id: string (unique identifier)
  task_type: string (pick_up, place, grasp, release, etc.)
  target_object: DetectedObject (object to manipulate)
  target_location: Pose (location for placement)
  gripper_configuration: string (gripper settings)
  execution_status: string (pending, executing, completed, failed)
  start_time: timestamp (when task started)
  completion_time: timestamp (when task completed)
  success: boolean (whether task was successful)
  error_code: string (error code if task failed)
  grasp_strategy: string (strategy used for grasping)
}
```

### Gripper State Model
```
GripperState {
  id: string (unique identifier)
  position: float (gripper position, 0-1)
  force: float (gripper force applied)
  status: string (open, closed, holding_object, error)
  object_held: DetectedObject (object currently held, if any)
  max_force: float (maximum force setting)
  max_position: float (maximum position setting)
  timestamp: timestamp (when state was recorded)
}
```

## System Integration Data Models

### Robot State Model
```
RobotState {
  id: string (unique identifier)
  position: Vector3 (current position x, y, z)
  orientation: Quaternion (current orientation)
  battery_level: float (battery level, 0-1)
  gripper_status: string (gripper status)
  current_task: string (current task being executed)
  held_object: DetectedObject (object currently held)
  navigation_status: string (current navigation status)
  manipulation_status: string (current manipulation status)
  perception_status: string (current perception status)
  timestamp: timestamp (when state was recorded)
}
```

### System Status Model
```
SystemStatus {
  id: string (unique identifier)
  component_status: ComponentStatus[] (status of each system component)
  overall_status: string (system-wide status: operational, degraded, failed)
  active_plans: string[] (IDs of currently active plans)
  error_conditions: string[] (current error conditions)
  performance_metrics: PerformanceMetrics (current performance metrics)
  timestamp: timestamp (when status was recorded)
}
```

### Component Status Model
```
ComponentStatus {
  component_name: string (name of the component)
  status: string (operational, degraded, failed, initializing)
  health_score: float (health score, 0-1)
  last_error: string (last error message, if any)
  uptime: float (component uptime in seconds)
  response_time: float (average response time)
}
```

### Performance Metrics Model
```
PerformanceMetrics {
  id: string (unique identifier)
  voice_processing_latency: float (average voice processing time)
  llm_response_time: float (average LLM response time)
  navigation_accuracy: float (navigation accuracy percentage)
  object_detection_rate: float (objects detected per second)
  task_completion_rate: float (successful tasks / total tasks)
  system_uptime: float (percentage of time system is operational)
  resource_usage: ResourceUsage (current resource usage)
  timestamp: timestamp (when metrics were recorded)
}
```

### Resource Usage Model
```
ResourceUsage {
  cpu_usage: float (CPU usage percentage)
  memory_usage: float (memory usage percentage)
  gpu_usage: float (GPU usage percentage, if applicable)
  network_usage: float (network usage in MB/s)
  disk_usage: float (disk usage percentage)
  timestamp: timestamp (when usage was recorded)
}
```

## Data Validation Rules

### Required Fields
- All entities must have a unique ID
- Timestamps must be in valid format
- Confidence values must be between 0 and 1
- Action steps must have valid action types

### Constraints
- Action step numbers must be sequential within a plan
- Navigation goals must have valid poses
- Detected objects must have valid bounding boxes
- Robot state positions must be within environment bounds

### Data Relationships
- Action steps must reference valid action plans
- Detected objects must reference valid perception results
- Navigation goals must reference valid robot states
- Manipulation tasks must reference valid detected objects