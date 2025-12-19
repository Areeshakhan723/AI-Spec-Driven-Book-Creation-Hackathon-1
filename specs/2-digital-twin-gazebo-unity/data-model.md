# Module 2: The Digital Twin (Gazebo & Unity) - Data Model

## Overview

This document defines the data models used in the digital twin simulation system, including robot models, sensor data structures, and simulation state representations.

## Robot Model Structure

### Robot Entity
```
Robot {
  id: string (unique identifier)
  name: string (human-readable name)
  type: string (e.g., "humanoid", "wheeled", "quadraped")
  description: string (detailed description)
  urdf_path: string (path to URDF file)
  sdf_path: string (path to SDF file, if different)
  unity_prefab: string (Unity prefab reference)

  // Physical properties
  mass: float (in kg)
  dimensions: Vector3 (width, height, depth in meters)
  center_of_mass: Vector3 (offset from origin)

  // Configuration
  joint_names: string[] (list of joint names)
  link_names: string[] (list of link names)
  sensor_names: string[] (list of sensor names)

  // State information
  position: Vector3 (x, y, z in world coordinates)
  orientation: Quaternion (rotation as quaternion)
  velocity: Vector3 (linear velocity)
  angular_velocity: Vector3 (angular velocity)

  // Timestamps
  created_at: timestamp
  updated_at: timestamp
}
```

### Joint Model
```
Joint {
  id: string (unique identifier)
  name: string (joint name)
  type: string (e.g., "revolute", "prismatic", "fixed", "continuous")
  parent_link: string (name of parent link)
  child_link: string (name of child link)

  // Joint limits
  lower_limit: float (minimum position, radians or meters)
  upper_limit: float (maximum position, radians or meters)
  effort_limit: float (maximum effort)
  velocity_limit: float (maximum velocity)

  // Current state
  position: float (current position)
  velocity: float (current velocity)
  effort: float (current effort)

  // Control parameters
  kp: float (proportional gain for position control)
  ki: float (integral gain for position control)
  kd: float (derivative gain for position control)
}
```

### Link Model
```
Link {
  id: string (unique identifier)
  name: string (link name)
  robot_id: string (reference to parent robot)

  // Inertial properties
  mass: float (mass in kg)
  inertia: Matrix3x3 (3x3 inertia matrix)
  center_of_mass: Vector3 (center of mass offset)

  // Visual properties
  visual_mesh: string (path to visual mesh)
  visual_material: string (material name or color)
  collision_mesh: string (path to collision mesh)

  // State information
  position: Vector3 (position in world coordinates)
  orientation: Quaternion (orientation as quaternion)
  velocity: Vector3 (linear velocity)
  angular_velocity: Vector3 (angular velocity)
}
```

## Sensor Models

### Sensor Base Model
```
Sensor {
  id: string (unique identifier)
  name: string (sensor name)
  type: string (e.g., "lidar", "camera", "imu", "gps", "force_torque")
  robot_id: string (reference to parent robot)
  link_name: string (name of link where sensor is attached)

  // Position and orientation relative to link
  position: Vector3 (position offset from link origin)
  orientation: Quaternion (orientation relative to link)

  // Configuration
  update_rate: float (update rate in Hz)
  enabled: boolean (whether sensor is active)

  // Data quality
  noise_model: string (type of noise model)
  noise_parameters: object (parameters for noise model)
  max_range: float (maximum sensing range, if applicable)
  min_range: float (minimum sensing range, if applicable)

  // ROS interface
  topic: string (ROS topic name)
  frame_id: string (ROS frame ID)
}
```

### LiDAR Sensor Model
```
LidarSensor extends Sensor {
  // LiDAR-specific properties
  samples: int (number of rays in horizontal scan)
  horizontal_resolution: float (angular resolution in radians)
  horizontal_min_angle: float (starting angle in radians)
  horizontal_max_angle: float (ending angle in radians)
  vertical_samples: int (number of rays in vertical scan, for 3D lidar)
  vertical_min_angle: float (vertical starting angle)
  vertical_max_angle: float (vertical ending angle)

  // Output data
  ranges: float[] (array of distance measurements)
  intensities: float[] (array of intensity measurements, if available)

  // Performance
  fov_horizontal: float (horizontal field of view in degrees)
  fov_vertical: float (vertical field of view in degrees, for 3D lidar)
}
```

### Camera Sensor Model
```
CameraSensor extends Sensor {
  // Camera-specific properties
  width: int (image width in pixels)
  height: int (image height in pixels)
  fov: float (field of view in degrees)
  format: string (e.g., "RGB8", "BGR8", "MONO8")

  // Intrinsic parameters
  fx: float (focal length x in pixels)
  fy: float (focal length y in pixels)
  cx: float (principal point x in pixels)
  cy: float (principal point y in pixels)
  k1: float (distortion coefficient 1)
  k2: float (distortion coefficient 2)
  p1: float (distortion coefficient 3)
  p2: float (distortion coefficient 4)

  // Output data
  image_data: uint8[] (raw image data)
  encoding: string (image encoding format)
  timestamp: timestamp (time of image capture)
}
```

### IMU Sensor Model
```
ImuSensor extends Sensor {
  // IMU-specific properties
  measure_orientation: boolean (whether to measure orientation)
  measure_angular_velocity: boolean (whether to measure angular velocity)
  measure_linear_acceleration: boolean (whether to measure linear acceleration)

  // Noise characteristics
  orientation_noise: float (orientation measurement noise in rad)
  angular_velocity_noise: float (angular velocity noise in rad/s)
  linear_acceleration_noise: float (linear acceleration noise in m/s²)

  // Output data
  orientation: Quaternion (measured orientation)
  angular_velocity: Vector3 (measured angular velocity)
  linear_acceleration: Vector3 (measured linear acceleration)

  // Covariance matrices
  orientation_covariance: float[9] (orientation measurement covariance)
  angular_velocity_covariance: float[9] (angular velocity covariance)
  linear_acceleration_covariance: float[9] (linear acceleration covariance)
}
```

## Simulation Environment Model

### World Model
```
World {
  id: string (unique identifier)
  name: string (world name)
  description: string (world description)
  sdf_path: string (path to SDF world file)

  // Physical properties
  gravity: Vector3 (gravity vector, typically [0, 0, -9.81])
  physics_engine: string (e.g., "ode", "bullet", "dart")
  real_time_factor: float (real-time update rate)

  // Entities in the world
  models: Model[] (list of models in the world)
  lights: Light[] (list of light sources)
  ground_planes: GroundPlane[] (list of ground planes)

  // Simulation state
  time: float (simulation time in seconds)
  paused: boolean (whether simulation is paused)
  step_size: float (simulation step size in seconds)
}
```

### Model Model
```
Model {
  id: string (unique identifier)
  name: string (model name)
  sdf_path: string (path to SDF model file)
  position: Vector3 (position in world coordinates)
  orientation: Quaternion (orientation as quaternion)

  // Model properties
  static: boolean (whether model is static)
  self_collide: boolean (whether model parts can collide with each other)
  canonical_link: string (name of canonical link)

  // Links and joints
  links: Link[] (list of links in the model)
  joints: Joint[] (list of joints in the model)
  sensors: Sensor[] (list of sensors in the model)
}
```

## Simulation State Model

### Simulation State
```
SimulationState {
  timestamp: timestamp (when state was recorded)
  world_time: float (simulation time)
  real_time_elapsed: float (real time elapsed since simulation start)

  // Robot states
  robot_states: RobotState[] (list of all robot states)

  // Sensor readings
  sensor_readings: SensorReading[] (list of all sensor readings)

  // Environment state
  world_state: WorldState (state of the simulation world)

  // Performance metrics
  real_time_factor: float (current real-time factor)
  update_rate: float (current update rate)
  cpu_usage: float (CPU usage percentage)
  memory_usage: float (memory usage in MB)
}
```

### Robot State
```
RobotState {
  robot_id: string (reference to robot)
  timestamp: timestamp (when state was recorded)

  // Pose information
  position: Vector3 (position in world coordinates)
  orientation: Quaternion (orientation as quaternion)
  pose_covariance: float[36] (pose covariance matrix)

  // Velocity information
  linear_velocity: Vector3 (linear velocity)
  angular_velocity: Vector3 (angular velocity)
  velocity_covariance: float[36] (velocity covariance matrix)

  // Joint states
  joint_positions: float[] (array of joint positions)
  joint_velocities: float[] (array of joint velocities)
  joint_efforts: float[] (array of joint efforts)

  // Control commands
  commanded_velocity: Twist (last commanded velocity)
  commanded_joints: JointCommand[] (last commanded joint positions/velocities)
}
```

## Data Validation Rules

### Required Fields
- All entities must have a unique ID
- Robot models must have valid URDF/SDF paths
- Sensor configurations must be within physical limits
- Timestamps must be in valid format

### Constraints
- Joint positions must be within defined limits
- Sensor ranges must be positive values
- Quaternion orientations must be normalized
- Update rates must be positive values

### Data Relationships
- Joint parent/child links must exist in the same robot
- Sensors must be attached to valid links
- Robot states must reference existing robots
- Simulation states must have valid timestamps