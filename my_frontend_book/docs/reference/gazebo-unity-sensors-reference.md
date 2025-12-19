---
sidebar_position: 8
title: "Gazebo & Unity Sensor Reference"
---

# Gazebo & Unity Sensor Reference

This reference guide provides comprehensive information about sensor simulation in both Gazebo and Unity environments for humanoid robotics applications.

## Gazebo Sensor Types

### LiDAR Sensors

LiDAR (Light Detection and Ranging) sensors are essential for navigation, mapping, and obstacle detection in robotics.

#### Basic LiDAR Configuration

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -π -->
        <max_angle>3.14159</max_angle>   <!-- π -->
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
```

#### LiDAR Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `samples` | Number of rays in the scan | 360-1440 |
| `min_angle` | Starting angle of scan (radians) | -π to π |
| `max_angle` | Ending angle of scan (radians) | -π to π |
| `min_range` | Minimum detection range (m) | 0.05-0.3 |
| `max_range` | Maximum detection range (m) | 10-100 |
| `resolution` | Angular resolution (radians) | 0.001-0.01 |

### Camera Sensors

Camera sensors provide visual information for perception tasks.

#### Basic Camera Configuration

```xml
<sensor name="camera" type="camera">
  <camera>
    <horizontal_fov>1.089</horizontal_fov>  <!-- 62.4 degrees -->
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
```

#### Camera Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `horizontal_fov` | Horizontal field of view (radians) | 0.7-1.57 (40°-90°) |
| `width` | Image width in pixels | 320, 640, 1280 |
| `height` | Image height in pixels | 240, 480, 960 |
| `format` | Color format | R8G8B8, RGB_INT8 |
| `near` | Near clipping plane (m) | 0.05-0.5 |
| `far` | Far clipping plane (m) | 10-100 |

### IMU Sensors

Inertial Measurement Unit (IMU) sensors provide orientation and acceleration data.

#### Basic IMU Configuration

```xml
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
```

#### IMU Parameters

| Parameter | Description | Typical Values |
|-----------|-------------|----------------|
| `update_rate` | Sensor update frequency (Hz) | 50-1000 |
| `angular_velocity_stddev` | Angular velocity noise (rad/s) | 0.001-0.01 |
| `linear_acceleration_stddev` | Linear acceleration noise (m/s²) | 0.01-0.1 |

## Unity Sensor Simulation

### Camera as Sensor

Unity cameras can be configured to simulate various types of visual sensors.

#### Basic Camera Sensor Script

```csharp
using UnityEngine;

public class CameraSensor : MonoBehaviour
{
    public Camera cameraComponent;
    public int width = 640;
    public int height = 480;
    public float fov = 60f;

    private RenderTexture renderTexture;

    void Start()
    {
        SetupCamera();
    }

    void SetupCamera()
    {
        cameraComponent = GetComponent<Camera>();
        cameraComponent.fieldOfView = fov;

        // Create render texture for sensor output
        renderTexture = new RenderTexture(width, height, 24);
        cameraComponent.targetTexture = renderTexture;
    }

    // Method to capture sensor data
    public Texture2D CaptureImage()
    {
        RenderTexture.active = renderTexture;
        Texture2D image = new Texture2D(width, height, TextureFormat.RGB24, false);
        image.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        image.Apply();
        RenderTexture.active = null;
        return image;
    }
}
```

### Depth Camera Simulation

Unity can simulate depth cameras using custom shaders.

#### Depth Camera Script

```csharp
using UnityEngine;

[ExecuteInEditMode]
public class DepthCamera : MonoBehaviour
{
    public Shader depthShader;
    private Camera cam;

    void Start()
    {
        cam = GetComponent<Camera>();
        cam.depthTextureMode = DepthTextureMode.Depth;
    }

    void OnEnable()
    {
        if (depthShader != null)
        {
            cam.SetReplacementShader(depthShader, "RenderType");
        }
    }

    void OnDisable()
    {
        cam.ResetReplacementShader();
    }
}
```

### LiDAR Simulation in Unity

LiDAR simulation in Unity uses raycasting to detect objects.

#### LiDAR Simulation Script

```csharp
using UnityEngine;
using System.Collections.Generic;

public class UnityLidar : MonoBehaviour
{
    [Header("LiDAR Configuration")]
    public int rays = 360;
    public float range = 30.0f;
    public float angle = 360.0f;
    public LayerMask layerMask = -1;

    private float[] lidarReadings;

    void Start()
    {
        lidarReadings = new float[rays];
    }

    void Update()
    {
        SimulateLidar();
    }

    void SimulateLidar()
    {
        Vector3 sensorPosition = transform.position;
        Vector3 sensorForward = transform.forward;

        for (int i = 0; i < rays; i++)
        {
            float angle = (i * angle / rays) * Mathf.Deg2Rad;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * sensorForward;

            RaycastHit hit;
            if (Physics.Raycast(sensorPosition, direction, out hit, range, layerMask))
            {
                lidarReadings[i] = hit.distance;
            }
            else
            {
                lidarReadings[i] = range; // No obstacle detected
            }
        }
    }

    public float[] GetReadings()
    {
        return lidarReadings;
    }
}
```

## Sensor Fusion Concepts

### Data Integration

Sensor fusion combines data from multiple sensors to improve accuracy and reliability.

#### Basic Sensor Fusion Approach

```python
import numpy as np
from scipy.spatial.transform import Rotation as R

class BasicSensorFusion:
    def __init__(self):
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.last_time = None

    def update_with_imu(self, angular_vel, linear_acc, timestamp):
        """Update state with IMU data"""
        if self.last_time is not None:
            dt = timestamp - self.last_time

            # Update orientation using angular velocity
            angle = np.linalg.norm(angular_vel) * dt
            if angle > 0:
                axis = angular_vel / np.linalg.norm(angular_vel)
                dq = np.array([
                    axis[0] * np.sin(angle/2),
                    axis[1] * np.sin(angle/2),
                    axis[2] * np.sin(angle/2),
                    np.cos(angle/2)
                ])
                self.orientation = self.quaternion_multiply(self.orientation, dq)
                self.orientation /= np.linalg.norm(self.orientation)

            # Update velocity and position using linear acceleration
            gravity = np.array([0, 0, 9.81])
            world_acc = R.from_quat(self.orientation).apply(linear_acc) - gravity
            self.velocity += world_acc * dt
            self.position += self.velocity * dt

        self.last_time = timestamp

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return np.array([w, x, y, z])
```

## Noise Modeling

### Adding Realistic Noise

Real sensors have inherent noise that should be modeled in simulation.

#### Noise Addition Functions

```python
import numpy as np

def add_lidar_noise(ranges, distance_dependent=True):
    """Add realistic noise to LiDAR ranges"""
    noise = np.random.normal(0, 0.01, ranges.shape)  # Base noise

    if distance_dependent:
        # Add distance-dependent noise (farther objects have more noise)
        noise += np.random.normal(0, 0.005 * ranges, ranges.shape)

    return ranges + noise

def add_camera_noise(image, noise_level=0.05):
    """Add noise to camera image"""
    noise = np.random.normal(0, noise_level, image.shape)
    noisy_image = image + noise
    return np.clip(noisy_image, 0, 1)  # Ensure values stay in [0,1] range

def add_imu_noise(angular_vel, linear_acc,
                  gyro_noise=0.001, accel_noise=0.017):
    """Add noise to IMU measurements"""
    noisy_angular_vel = angular_vel + np.random.normal(0, gyro_noise, angular_vel.shape)
    noisy_linear_acc = linear_acc + np.random.normal(0, accel_noise, linear_acc.shape)
    return noisy_angular_vel, noisy_linear_acc
```

## Performance Considerations

### Optimization Tips

1. **LiDAR Performance**:
   - Reduce ray count for real-time applications
   - Use spatial partitioning for raycasting
   - Implement multi-threading for parallel ray processing

2. **Camera Performance**:
   - Use appropriate resolution for application needs
   - Implement frustum culling to avoid rendering invisible objects
   - Use texture compression for memory efficiency

3. **IMU Performance**:
   - Use appropriate update rates (typically 100-1000 Hz)
   - Implement efficient noise generation algorithms
   - Use fixed time steps for integration

## Validation Metrics

### Sensor Quality Assessment

#### LiDAR Quality Metrics

```python
def evaluate_lidar_quality(scan_data):
    """Evaluate LiDAR data quality"""
    ranges = np.array(scan_data.ranges)
    valid_ranges = ranges[np.isfinite(ranges)]

    metrics = {
        'valid_ratio': len(valid_ranges) / len(ranges),
        'avg_range': np.mean(valid_ranges) if len(valid_ranges) > 0 else 0,
        'range_variance': np.var(valid_ranges) if len(valid_ranges) > 1 else 0,
        'min_distance': np.min(valid_ranges) if len(valid_ranges) > 0 else float('inf'),
        'max_distance': np.max(valid_ranges) if len(valid_ranges) > 0 else 0
    }

    return metrics
```

#### Camera Quality Metrics

```python
def evaluate_camera_quality(image):
    """Evaluate camera image quality"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate metrics
    mean_brightness = np.mean(gray)
    std_brightness = np.std(gray)
    sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()

    metrics = {
        'brightness': mean_brightness,
        'contrast': std_brightness,
        'sharpness': sharpness,
        'resolution': (image.shape[1], image.shape[0])
    }

    return metrics
```

This reference provides comprehensive information for implementing and working with sensor simulation in both Gazebo and Unity environments for digital twin applications in robotics.