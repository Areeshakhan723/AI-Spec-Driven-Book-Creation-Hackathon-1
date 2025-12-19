# Module 2: The Digital Twin (Gazebo & Unity) - API Contracts

## Overview

This document defines the API contracts for the digital twin simulation system, including interfaces between Gazebo, Unity, and ROS 2 components.

## Sensor Data Interfaces

### LiDAR Sensor Interface

#### Gazebo LiDAR Publisher
- **Topic**: `/lidar/scan`
- **Type**: `sensor_msgs/LaserScan`
- **Frequency**: 10 Hz

**Message Structure**:
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities
```

#### Camera Sensor Interface
- **Topic**: `/camera/image_raw`
- **Type**: `sensor_msgs/Image`
- **Frequency**: 30 Hz

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

#### IMU Sensor Interface
- **Topic**: `/imu/data`
- **Type**: `sensor_msgs/Imu`
- **Frequency**: 100 Hz

**Message Structure**:
```
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

## Robot Control Interfaces

### Velocity Command Interface
- **Topic**: `/cmd_vel`
- **Type**: `geometry_msgs/Twist`

**Message Structure**:
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

### Joint State Interface
- **Topic**: `/joint_states`
- **Type**: `sensor_msgs/JointState`

**Message Structure**:
```
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
```

## Simulation Control Interfaces

### Gazebo Services

#### Spawn Model Service
- **Service**: `/spawn_entity`
- **Type**: `gazebo_msgs/SpawnEntity`

**Request**:
```
string name
string xml
string robot_namespace
geometry_msgs/Pose initial_pose
string reference_frame
```

**Response**:
```
bool success
string status_message
```

#### Delete Model Service
- **Service**: `/delete_entity`
- **Type**: `gazebo_msgs/DeleteEntity`

**Request**:
```
string name
```

**Response**:
```
bool success
string status_message
```

## Unity-ROS Communication

### Unity ROS# Interface

#### Publisher Configuration
- **Protocol**: WebSocket
- **Default Port**: 9090
- **Message Types**: All standard ROS message types

#### Quality of Service Settings
- **Reliability**: Best effort or Reliable
- **Durability**: Volatile
- **History**: Keep last N messages

## Validation and Monitoring Interfaces

### Sensor Quality Publishers

#### LiDAR Quality Score
- **Topic**: `/lidar/quality_score`
- **Type**: `std_msgs/Float32`

#### Camera Quality Score
- **Topic**: `/camera/quality_score`
- **Type**: `std_msgs/Float32`

### Fused Odometry
- **Topic**: `/fused_odom`
- **Type**: `nav_msgs/Odometry`

**Message Structure**:
```
std_msgs/Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

## Error Handling

### Standard Error Responses
- **Success**: `bool success = true`
- **Failure**: `bool success = false` with descriptive `status_message`
- **Timeout**: Standard ROS service timeout (typically 3-10 seconds)

### Common Error Codes
- `0`: Success
- `1`: Invalid parameters
- `2`: Resource not available
- `3`: Communication error
- `4`: Timeout
- `5`: Internal error

## Performance Requirements

### Latency Requirements
- Sensor data: < 100ms end-to-end
- Control commands: < 50ms end-to-end
- Service calls: < 1000ms

### Throughput Requirements
- LiDAR: Up to 1440 points per scan at 10Hz
- Camera: Up to 640x480 RGB at 30Hz
- IMU: Up to 1000Hz data rate

### Reliability
- 99.9% message delivery for critical control interfaces
- 99% message delivery for sensor interfaces
- Automatic reconnection for WebSocket connections