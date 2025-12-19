---
sidebar_position: 7
title: "Sensor Simulation Tutorial"
---

# Sensor Simulation Tutorial

This tutorial will guide you through implementing and validating various sensor simulations in both Gazebo and Unity environments, focusing on LiDAR, cameras, and IMU sensors for humanoid robots.

## Prerequisites

Before starting this tutorial, you should:
- Have completed the Gazebo Simulation Tutorial
- Have completed the Unity Digital Twin Tutorial
- Understand basic ROS 2 concepts
- Have basic programming knowledge in Python and C#

## Setting up Sensor Simulation Environment

### Step 1: Create a Multi-Sensor Robot Model

First, let's create a robot model with multiple sensors. Create a URDF file called `sensor_robot.urdf`:

```xml
<?xml version="1.0"?>
<robot name="sensor_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="head_joint" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25"/>
  </joint>

  <!-- LiDAR sensor -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
  </joint>

  <!-- Camera sensor -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.05"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0.08 0 0" rpy="0 0 0"/>
  </joint>

  <!-- IMU sensor -->
  <link name="imu_link">
    <inertial>
      <mass value="0.01"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0"/>
  </joint>

  <!-- Gazebo sensor definitions -->
  <gazebo reference="lidar_link">
    <sensor name="lidar" type="ray">
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>/lidar</namespace>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="camera" type="camera">
      <camera>
        <horizontal_fov>1.089</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
        <ros>
          <namespace>/camera</namespace>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <imu>
        <angular_velocity>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.001</stddev>
            </noise>
          </z>
        </angular_velocity>
        <linear_acceleration>
          <x>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </x>
          <y>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </y>
          <z>
            <noise type="gaussian">
              <mean>0.0</mean>
              <stddev>0.017</stddev>
            </noise>
          </z>
        </linear_acceleration>
      </imu>
      <plugin name="imu_controller" filename="libgazebo_ros_imu.so">
        <ros>
          <namespace>/imu</namespace>
        </ros>
      </plugin>
    </sensor>
  </gazebo>

  <!-- Gazebo ROS control plugin -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros2_control.so">
      <parameters>$(find sensor_robot_description)/config/robot_control.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```

### Step 2: Create a Sensor Validation Node

Create a Python script called `sensor_validator.py` to validate sensor data:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float32
import numpy as np
import cv2
from cv_bridge import CvBridge
import matplotlib.pyplot as plt

class SensorValidator(Node):
    def __init__(self):
        super().__init__('sensor_validator')

        # Create subscribers for different sensors
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_raw', self.camera_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Create publishers for validation results
        self.lidar_quality_pub = self.create_publisher(
            Float32, '/lidar/quality_score', 10)
        self.camera_quality_pub = self.create_publisher(
            Float32, '/camera/quality_score', 10)

        # Initialize data storage
        self.bridge = CvBridge()
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None

        # Statistics
        self.lidar_ranges_count = 0
        self.camera_frames_count = 0
        self.imu_samples_count = 0

        self.get_logger().info('Sensor Validator Node Started')

    def lidar_callback(self, msg):
        """Process LiDAR data and validate quality"""
        self.lidar_data = msg
        self.lidar_ranges_count += 1

        # Validate LiDAR data
        quality_score = self.validate_lidar_data(msg)

        # Publish quality score
        quality_msg = Float32()
        quality_msg.data = quality_score
        self.lidar_quality_pub.publish(quality_msg)

        # Log statistics periodically
        if self.lidar_ranges_count % 100 == 0:
            self.get_logger().info(
                f'LiDAR: {self.lidar_ranges_count} samples, '
                f'Quality: {quality_score:.2f}')

    def camera_callback(self, msg):
        """Process camera data and validate quality"""
        self.camera_data = msg
        self.camera_frames_count += 1

        # Validate camera data
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            quality_score = self.validate_camera_data(cv_image)

            # Publish quality score
            quality_msg = Float32()
            quality_msg.data = quality_score
            self.camera_quality_pub.publish(quality_msg)

            if self.camera_frames_count % 100 == 0:
                self.get_logger().info(
                    f'Camera: {self.camera_frames_count} frames, '
                    f'Quality: {quality_score:.2f}')
        except Exception as e:
            self.get_logger().error(f'Camera processing error: {e}')

    def imu_callback(self, msg):
        """Process IMU data and validate quality"""
        self.imu_data = msg
        self.imu_samples_count += 1

        # Validate IMU data
        quality_score = self.validate_imu_data(msg)

        if self.imu_samples_count % 100 == 0:
            self.get_logger().info(
                f'IMU: {self.imu_samples_count} samples, '
                f'Angular Vel: [{msg.angular_velocity.x:.3f}, '
                f'{msg.angular_velocity.y:.3f}, '
                f'{msg.angular_velocity.z:.3f}], '
                f'Linear Acc: [{msg.linear_acceleration.x:.3f}, '
                f'{msg.linear_acceleration.y:.3f}, '
                f'{msg.linear_acceleration.z:.3f}]')

    def validate_lidar_data(self, scan_msg):
        """Validate LiDAR data quality"""
        ranges = np.array(scan_msg.ranges)

        # Filter out invalid ranges (inf, nan)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            return 0.0

        # Calculate quality metrics
        avg_range = np.mean(valid_ranges) if len(valid_ranges) > 0 else 0
        range_variance = np.var(valid_ranges) if len(valid_ranges) > 1 else 0

        # Quality score based on reasonable range values
        # Higher score for more valid readings and reasonable variance
        valid_ratio = len(valid_ranges) / len(ranges)
        quality = min(1.0, valid_ratio * 2.0)  # Cap at 1.0

        return quality

    def validate_camera_data(self, image):
        """Validate camera data quality"""
        # Check if image is too dark or too bright
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calculate histogram and statistics
        mean_brightness = np.mean(gray)
        std_brightness = np.std(gray)

        # Check for reasonable brightness (not too dark or too bright)
        brightness_score = 1.0 - min(abs(mean_brightness - 128) / 128.0, 1.0)

        # Check for reasonable contrast
        contrast_score = min(std_brightness / 50.0, 1.0)

        # Overall quality score
        quality = (brightness_score + contrast_score) / 2.0

        return quality

    def validate_imu_data(self, imu_msg):
        """Validate IMU data quality"""
        # Check if values are within reasonable ranges
        # For a stationary robot, angular velocities should be near 0
        # Linear accelerations should be around 9.8 m/s² for Z-axis (gravity)

        ang_vel_mag = np.sqrt(
            imu_msg.angular_velocity.x**2 +
            imu_msg.angular_velocity.y**2 +
            imu_msg.angular_velocity.z**2
        )

        lin_acc_mag = np.sqrt(
            imu_msg.linear_acceleration.x**2 +
            imu_msg.linear_acceleration.y**2 +
            imu_msg.linear_acceleration.z**2
        )

        # Reasonable thresholds (adjust based on your robot)
        max_ang_vel = 10.0  # rad/s
        reasonable_gravity = 9.8  # m/s² ± 2

        ang_vel_score = max(0, 1 - ang_vel_mag / max_ang_vel)
        gravity_score = max(0, 1 - abs(lin_acc_mag - reasonable_gravity) / 2.0)

        quality = (ang_vel_score + gravity_score) / 2.0
        return min(quality, 1.0)

def main(args=None):
    rclpy.init(args=args)
    validator = SensorValidator()

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Shutting down Sensor Validator')
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create a Sensor Fusion Node

Create a Python script called `sensor_fusion.py` to combine data from multiple sensors:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Create subscribers for sensors
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Create publisher for fused odometry
        self.odom_pub = self.create_publisher(Odometry, '/fused_odom', 10)

        # Initialize state variables
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        # Time tracking
        self.last_time = self.get_clock().now()

        # Covariance matrices (simplified)
        self.pose_covariance = np.eye(6) * 0.1
        self.twist_covariance = np.eye(6) * 0.1

        self.get_logger().info('Sensor Fusion Node Started')

    def lidar_callback(self, msg):
        """Process LiDAR data for environment mapping"""
        # Extract valid ranges
        ranges = np.array(msg.ranges)
        valid_ranges = np.isfinite(ranges)

        if not np.any(valid_ranges):
            return

        # Calculate some features from LiDAR data
        min_distance = np.min(ranges[valid_ranges]) if np.any(valid_ranges) else float('inf')

        # Use minimum distance as a simple obstacle detection
        if min_distance < 1.0:  # Obstacle within 1 meter
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')

    def imu_callback(self, msg):
        """Process IMU data for orientation and acceleration"""
        # Update angular velocity from IMU
        self.angular_velocity = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Get linear acceleration (in sensor frame)
        linear_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        # Convert acceleration to world frame using current orientation
        rotation = R.from_quat(self.orientation)
        world_acc = rotation.apply(linear_acc)

        # Remove gravity from linear acceleration
        gravity = np.array([0, 0, 9.81])
        linear_acc_world = world_acc - gravity

        # Update velocity and position using simple integration
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt > 0:
            # Update velocity
            self.velocity += linear_acc_world * dt

            # Update position
            self.position += self.velocity * dt

            # Update orientation (simplified - in practice, use proper integration)
            # For small dt, we can approximate the rotation
            angle = np.linalg.norm(self.angular_velocity) * dt
            if angle > 0:
                axis = self.angular_velocity / np.linalg.norm(self.angular_velocity)

                # Create quaternion for rotation
                sin_half = np.sin(angle / 2)
                cos_half = np.cos(angle / 2)

                dq = np.array([
                    axis[0] * sin_half,
                    axis[1] * sin_half,
                    axis[2] * sin_half,
                    cos_half
                ])

                # Apply rotation
                self.orientation = self.quaternion_multiply(self.orientation, dq)
                # Normalize quaternion
                self.orientation = self.orientation / np.linalg.norm(self.orientation)

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])

    def publish_odometry(self):
        """Publish fused odometry data"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set position
        odom_msg.pose.pose.position.x = self.position[0]
        odom_msg.pose.pose.position.y = self.position[1]
        odom_msg.pose.pose.position.z = self.position[2]

        # Set orientation
        odom_msg.pose.pose.orientation.w = self.orientation[3]
        odom_msg.pose.pose.orientation.x = self.orientation[0]
        odom_msg.pose.pose.orientation.y = self.orientation[1]
        odom_msg.pose.pose.orientation.z = self.orientation[2]

        # Set velocity
        odom_msg.twist.twist.linear.x = self.velocity[0]
        odom_msg.twist.twist.linear.y = self.velocity[1]
        odom_msg.twist.twist.linear.z = self.velocity[2]

        odom_msg.twist.twist.angular.x = self.angular_velocity[0]
        odom_msg.twist.twist.angular.y = self.angular_velocity[1]
        odom_msg.twist.twist.angular.z = self.angular_velocity[2]

        # Set covariance
        odom_msg.pose.covariance = self.pose_covariance.flatten().tolist()
        odom_msg.twist.covariance = self.twist_covariance.flatten().tolist()

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    fusion_node = SensorFusionNode()

    # Timer to publish odometry at regular intervals
    timer = fusion_node.create_timer(0.1, fusion_node.publish_odometry)  # 10 Hz

    try:
        rclpy.spin(fusion_node)
    except KeyboardInterrupt:
        fusion_node.get_logger().info('Shutting down Sensor Fusion Node')
    finally:
        fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Unity Sensor Simulation

### Step 1: Create Sensor Simulation in Unity

Create a C# script called `SensorSimulator.cs`:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SensorSimulator : MonoBehaviour
{
    [Header("Sensor Configuration")]
    public bool enableLidar = true;
    public bool enableCamera = true;
    public bool enableIMU = true;

    [Header("LiDAR Settings")]
    public int lidarRays = 360;
    public float lidarRange = 30.0f;
    public float lidarAngle = 360.0f;
    public LayerMask lidarLayerMask = -1;

    [Header("Camera Settings")]
    public Camera sensorCamera;
    public int cameraWidth = 640;
    public int cameraHeight = 480;
    public float cameraFOV = 60.0f;

    [Header("IMU Settings")]
    public float imuUpdateRate = 100.0f; // Hz
    public float angularVelocityNoise = 0.001f;
    public float linearAccelerationNoise = 0.017f;

    // Sensor data storage
    private float[] lidarReadings;
    private RenderTexture cameraTexture;
    private Vector3 imuAngularVelocity;
    private Vector3 imuLinearAcceleration;
    private float imuLastUpdate;

    // For visualization
    private LineRenderer lidarVisualizer;

    void Start()
    {
        InitializeSensors();
    }

    void InitializeSensors()
    {
        if (enableLidar)
        {
            InitializeLidar();
        }

        if (enableCamera)
        {
            InitializeCamera();
        }

        if (enableIMU)
        {
            InitializeIMU();
        }
    }

    void InitializeLidar()
    {
        lidarReadings = new float[lidarRays];

        // Create line renderer for visualization
        lidarVisualizer = gameObject.AddComponent<LineRenderer>();
        lidarVisualizer.material = new Material(Shader.Find("Sprites/Default"));
        lidarVisualizer.widthMultiplier = 0.05f;
        lidarVisualizer.positionCount = lidarRays;
    }

    void InitializeCamera()
    {
        if (sensorCamera == null)
        {
            sensorCamera = GetComponent<Camera>();
        }

        if (sensorCamera == null)
        {
            sensorCamera = gameObject.AddComponent<Camera>();
        }

        sensorCamera.fieldOfView = cameraFOV;
        cameraTexture = new RenderTexture(cameraWidth, cameraHeight, 24);
        sensorCamera.targetTexture = cameraTexture;
    }

    void InitializeIMU()
    {
        imuLastUpdate = Time.time;
    }

    void Update()
    {
        if (enableLidar)
        {
            UpdateLidar();
        }

        if (enableIMU)
        {
            UpdateIMU();
        }
    }

    void UpdateLidar()
    {
        Vector3 sensorPosition = transform.position;
        Vector3 sensorForward = transform.forward;

        for (int i = 0; i < lidarRays; i++)
        {
            float angle = (i * lidarAngle / lidarRays) * Mathf.Deg2Rad;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * sensorForward;

            RaycastHit hit;
            if (Physics.Raycast(sensorPosition, direction, out hit, lidarRange, lidarLayerMask))
            {
                lidarReadings[i] = hit.distance;
            }
            else
            {
                lidarReadings[i] = lidarRange; // No obstacle detected
            }

            // Update visualization
            if (lidarVisualizer != null)
            {
                lidarVisualizer.SetPosition(i, sensorPosition + direction * lidarReadings[i]);
            }
        }
    }

    void UpdateIMU()
    {
        float deltaTime = Time.time - imuLastUpdate;
        if (deltaTime >= 1.0f / imuUpdateRate)
        {
            // Get angular velocity from the object's rotation
            imuAngularVelocity = GetAngularVelocity();

            // Get linear acceleration (approximation)
            imuLinearAcceleration = GetLinearAcceleration();

            // Add noise to simulate real sensor behavior
            AddNoiseToIMU();

            // Send data via ROS or other communication method
            PublishIMUData();

            imuLastUpdate = Time.time;
        }
    }

    Vector3 GetAngularVelocity()
    {
        // Approximate angular velocity from rotation change
        // In practice, this would come from Unity's physics system
        return new Vector3(
            Random.Range(-0.1f, 0.1f),
            Random.Range(-0.1f, 0.1f),
            Random.Range(-0.1f, 0.1f)
        );
    }

    Vector3 GetLinearAcceleration()
    {
        // Get linear acceleration (gravity + movement)
        // In practice, this would consider the object's movement and gravity
        Vector3 gravity = Physics.gravity;
        Vector3 acceleration = gravity + (transform.position - transform.position) / Time.deltaTime; // Simplified
        return acceleration;
    }

    void AddNoiseToIMU()
    {
        // Add Gaussian noise to simulate sensor imperfections
        imuAngularVelocity += Random.insideUnitSphere * angularVelocityNoise;
        imuLinearAcceleration += Random.insideUnitSphere * linearAccelerationNoise;
    }

    void PublishIMUData()
    {
        // In a real implementation, this would publish to ROS
        // For now, we'll just log the data
        Debug.Log($"IMU: Angular={imuAngularVelocity}, Linear={imuLinearAcceleration}");
    }

    // Public methods to access sensor data
    public float[] GetLidarReadings()
    {
        return lidarReadings;
    }

    public Texture2D GetCameraImage()
    {
        if (sensorCamera == null || cameraTexture == null)
            return null;

        // Capture the camera texture to a Texture2D
        RenderTexture.active = cameraTexture;
        Texture2D image = new Texture2D(cameraWidth, cameraHeight, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, cameraWidth, cameraHeight), 0, 0);
        image.Apply();
        RenderTexture.active = null;

        return image;
    }

    public Vector3 GetIMUAngularVelocity()
    {
        return imuAngularVelocity;
    }

    public Vector3 GetIMULinearAcceleration()
    {
        return imuLinearAcceleration;
    }

    // Method to visualize sensor data
    public void ToggleLidarVisualization(bool visible)
    {
        if (lidarVisualizer != null)
        {
            lidarVisualizer.enabled = visible;
        }
    }
}
```

### Step 2: Create Sensor Data Visualization

Create a C# script called `SensorDataVisualizer.cs`:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class SensorDataVisualizer : MonoBehaviour
{
    [Header("UI References")]
    public Text lidarStatusText;
    public Text cameraStatusText;
    public Text imuStatusText;
    public RawImage cameraFeedImage;
    public Slider lidarRangeSlider;
    public Toggle lidarVisualizationToggle;

    [Header("Sensor Simulator Reference")]
    public SensorSimulator sensorSimulator;

    [Header("Visualization Settings")]
    public float updateInterval = 0.1f;

    private float lastUpdateTime;

    void Start()
    {
        SetupUI();
    }

    void SetupUI()
    {
        if (lidarVisualizationToggle != null)
        {
            lidarVisualizationToggle.onValueChanged.AddListener(OnLidarVisualizationToggle);
        }

        if (lidarRangeSlider != null)
        {
            lidarRangeSlider.onValueChanged.AddListener(OnLidarRangeChanged);
        }
    }

    void Update()
    {
        if (Time.time - lastUpdateTime >= updateInterval)
        {
            UpdateSensorStatus();
            lastUpdateTime = Time.time;
        }
    }

    void UpdateSensorStatus()
    {
        if (sensorSimulator == null) return;

        // Update LiDAR status
        if (lidarStatusText != null)
        {
            float[] lidarData = sensorSimulator.GetLidarReadings();
            if (lidarData != null && lidarData.Length > 0)
            {
                float minRange = Mathf.Min(lidarData);
                float maxRange = Mathf.Max(lidarData);
                lidarStatusText.text = $"LiDAR: Min={minRange:F2}m, Max={maxRange:F2}m, Rays={lidarData.Length}";
            }
        }

        // Update Camera status
        if (cameraStatusText != null)
        {
            Texture2D cameraImage = sensorSimulator.GetCameraImage();
            if (cameraImage != null)
            {
                cameraStatusText.text = $"Camera: {cameraImage.width}x{cameraImage.height}";

                // Update camera feed display
                if (cameraFeedImage != null)
                {
                    cameraFeedImage.texture = cameraImage;
                }
            }
        }

        // Update IMU status
        if (imuStatusText != null)
        {
            Vector3 angVel = sensorSimulator.GetIMUAngularVelocity();
            Vector3 linAcc = sensorSimulator.GetIMULinearAcceleration();
            imuStatusText.text = $"IMU: AngVel=({angVel.x:F3},{angVel.y:F3},{angVel.z:F3}), LinAcc=({linAcc.x:F3},{linAcc.y:F3},{linAcc.z:F3})";
        }
    }

    void OnLidarVisualizationToggle(bool isOn)
    {
        if (sensorSimulator != null)
        {
            sensorSimulator.ToggleLidarVisualization(isOn);
        }
    }

    void OnLidarRangeChanged(float range)
    {
        // In a complete implementation, this would update the sensor simulator
        // For now, we'll just log the change
        Debug.Log($"LiDAR range changed to: {range}m");
    }
}
```

## Running the Simulation

### Step 1: Launch Gazebo with Sensor Robot

Create a launch file called `sensor_simulation.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'sensor_humanoid'
        ],
        output='screen'
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Sensor validator node
    sensor_validator = Node(
        package='sensor_validation',
        executable='sensor_validator',
        name='sensor_validator',
        output='screen'
    )

    # Sensor fusion node
    sensor_fusion = Node(
        package='sensor_fusion',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=[PathJoinSubstitution([
                FindPackageShare('sensor_robot_description'),
                'worlds',
                'simple_world.sdf'
            ])],
            description='Choose one of the world files from `/sensor_robot_description/worlds`'
        ),
        gazebo,
        spawn_entity,
        robot_state_publisher,
        sensor_validator,
        sensor_fusion
    ])
```

### Step 2: Run the Simulation

1. Build and source your ROS 2 workspace:
```bash
cd ~/sensor_simulation_ws
colcon build
source install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch sensor_robot_description sensor_simulation.launch.py
```

3. In another terminal, visualize the data:
```bash
# View LiDAR data
ros2 run rviz2 rviz2

# View camera feed
ros2 run rqt_image_view rqt_image_view

# Monitor sensor quality scores
ros2 topic echo /lidar/quality_score
```

## Validation and Testing

### Step 1: Create Validation Scripts

Create a validation script called `validate_simulation.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class SimulationValidator(Node):
    def __init__(self):
        super().__init__('simulation_validator')

        # Subscribers for sensor data
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # Quality score subscribers
        self.lidar_quality_sub = self.create_subscription(
            Float32, '/lidar/quality_score', self.lidar_quality_callback, 10)

        # Data storage
        self.lidar_data_history = deque(maxlen=100)
        self.imu_data_history = deque(maxlen=100)
        self.quality_scores = deque(maxlen=100)

        # Validation thresholds
        self.min_quality_threshold = 0.7
        self.max_lidar_noise = 0.1

        # Statistics
        self.total_samples = 0
        self.valid_samples = 0

        self.get_logger().info('Simulation Validator Started')

    def lidar_callback(self, msg):
        """Process LiDAR data for validation"""
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) > 0:
            # Calculate noise metrics
            range_variance = np.var(valid_ranges) if len(valid_ranges) > 1 else 0

            # Store for history
            self.lidar_data_history.append({
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                'valid_count': len(valid_ranges),
                'avg_range': np.mean(valid_ranges),
                'variance': range_variance
            })

            # Check if data is reasonable
            if range_variance < self.max_lidar_noise:
                self.valid_samples += 1

            self.total_samples += 1

    def imu_callback(self, msg):
        """Process IMU data for validation"""
        # Store IMU data for validation
        self.imu_data_history.append({
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            'angular_velocity': np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ]),
            'linear_acceleration': np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
        })

    def lidar_quality_callback(self, msg):
        """Process quality scores"""
        self.quality_scores.append({
            'timestamp': self.get_clock().now().nanoseconds / 1e9,
            'score': msg.data
        })

        # Check if quality is acceptable
        if msg.data >= self.min_quality_threshold:
            self.valid_samples += 1
        self.total_samples += 1

    def print_validation_report(self):
        """Print validation summary"""
        if self.total_samples > 0:
            success_rate = self.valid_samples / self.total_samples
            self.get_logger().info(
                f'Validation Report:\n'
                f'  Total samples: {self.total_samples}\n'
                f'  Valid samples: {self.valid_samples}\n'
                f'  Success rate: {success_rate:.2%}\n'
                f'  Quality threshold: {self.min_quality_threshold}'
            )
        else:
            self.get_logger().info('No samples collected yet')

def main(args=None):
    rclpy.init(args=args)
    validator = SimulationValidator()

    # Print validation report every 10 seconds
    timer = validator.create_timer(10.0, lambda: validator.print_validation_report())

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Shutting down Simulation Validator')
        validator.print_validation_report()
    finally:
        validator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

In this tutorial, you learned how to:
- Create a multi-sensor robot model with LiDAR, camera, and IMU
- Implement sensor validation in ROS 2
- Create sensor fusion algorithms
- Simulate sensors in Unity with visualization
- Validate simulation quality and accuracy
- Create tools for monitoring and analyzing sensor data

This provides a comprehensive foundation for sensor simulation in both Gazebo and Unity environments, allowing you to test and validate robotics algorithms before deployment on real hardware.