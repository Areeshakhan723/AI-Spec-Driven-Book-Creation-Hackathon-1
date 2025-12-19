---
title: ROS 2 Concepts Reference
description: Comprehensive reference for ROS 2 core concepts and terminology
sidebar_label: ROS 2 Concepts Reference
sidebar_position: 7
tags: [ros2, reference, concepts, terminology]
authors: [Spec-Kit Plus Team]
keywords: [ROS 2, concepts, reference, terminology, middleware]
---

# ROS 2 Concepts Reference

This reference provides comprehensive information about core ROS 2 concepts, terminology, and best practices. Use this as a quick lookup for ROS 2 fundamentals.

## Core Concepts

### Node
A node is an executable that uses the ROS 2 client library. Nodes are the fundamental building blocks of a ROS 2 system.

**Key Properties:**
- Performs specific tasks or functions
- Communicates with other nodes through topics, services, etc.
- Runs as a separate process
- Can be written in different programming languages

### Topic
A topic is a named bus over which nodes exchange messages using the publisher-subscriber communication pattern.

**Characteristics:**
- Asynchronous communication
- Many-to-many: multiple publishers and subscribers
- Unidirectional: data flows one way
- Anonymous: publishers and subscribers don't know about each other

### Publisher
A publisher sends messages to a topic.

**Usage:**
- Created with `create_publisher()` in client libraries
- Publishes messages using the `publish()` method
- Messages are serialized for network transmission

### Subscriber
A subscriber receives messages from a topic.

**Usage:**
- Created with `create_subscription()` in client libraries
- Processes messages via callback functions
- Runs callbacks when messages arrive

### Service
A service provides synchronous request-response communication.

**Components:**
- **Service Client**: Sends requests and waits for responses
- **Service Server**: Receives requests and sends responses
- **Service Interface**: Defines request and response message types

### Action
An action handles long-running operations with feedback and goal management.

**Components:**
- **Goal**: Request for a long-running operation
- **Feedback**: Interim results during operation
- **Result**: Final outcome when operation completes
- **Cancelation**: Ability to cancel ongoing operations

## Client Libraries

### rclpy
The Python client library for ROS 2.

**Key Features:**
- Object-oriented interface
- Integration with Python's asyncio
- Automatic memory management
- Rich set of built-in message types

### rclcpp
The C++ client library for ROS 2.

**Key Features:**
- High-performance implementation
- RAII-based resource management
- Template-based message handling
- Direct memory access for performance

### Other Client Libraries
- **rclnodejs**: JavaScript/TypeScript
- **rclrs**: Rust
- **rclc**: C
- **rcljava**: Java

## Message Types

### Standard Message Types
ROS 2 provides many built-in message types:

**Primitive Types:**
- `bool`
- `byte`
- `char`
- `float32`, `float64`
- `int8`, `int16`, `int32`, `int64`
- `uint8`, `uint16`, `uint32`, `uint64`
- `string`
- `wstring`

**Complex Types:**
- `std_msgs`: Basic types like String, Int32, Float64, etc.
- `geometry_msgs`: Pose, Twist, Vector3, etc.
- `sensor_msgs`: LaserScan, Image, JointState, etc.
- `nav_msgs`: OccupancyGrid, Path, Odometry, etc.

### Custom Message Types
You can define your own message types using `.msg` files:

```
# MyCustomMessage.msg
string name
int32 id
float64[] values
geometry_msgs/Pose pose
```

## Quality of Service (QoS)

QoS policies allow you to configure how messages are delivered in DDS.

### Reliability Policy
- **RELIABLE**: All messages are delivered (if possible)
- **BEST_EFFORT**: Messages may be dropped

### Durability Policy
- **TRANSIENT_LOCAL**: Late joiners receive previous messages
- **VOLATILE**: Late joiners only receive new messages

### Deadline Policy
Maximum time between data updates.

### Lifespan Policy
Maximum time a message is valid after publication.

### Liveliness Policy
How to detect if a participant is alive.

### History Policy
How many samples to keep for late joiners.

## Launch Files

Launch files allow you to start multiple nodes at once.

### Python Launch Files
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_package',
            executable='my_node',
            name='my_node_name',
            parameters=[
                {'param1': 'value1'},
                {'param2': 42}
            ]
        )
    ])
```

### XML Launch Files
```xml
<launch>
  <node pkg="my_package" exec="my_node" name="my_node_name">
    <param name="param1" value="value1"/>
    <param name="param2" value="42"/>
  </node>
</launch>
```

## Parameters

Parameters allow you to configure nodes at runtime.

### Setting Parameters
```python
# In a node
self.declare_parameter('my_param', 'default_value')
param_value = self.get_parameter('my_param').value
```

### Command Line
```bash
ros2 run my_package my_node --ros-args -p my_param:=value
```

## Services and Actions

### Service Interface Definition
Service files use `.srv` extension:

```
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Action Interface Definition
Action files use `.action` extension:

```
# Fibonacci.action
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

## TF (Transforms)

TF provides a way to keep track of coordinate frame relationships over time.

### Key Concepts
- **Frames**: Coordinate systems attached to objects
- **Transforms**: Relationships between frames
- **Tree Structure**: All frames connected to a common tree

### Usage
```python
# Publishing transforms
import tf2_ros
from geometry_msgs.msg import TransformStamped

br = tf2_ros.TransformBroadcaster(node)
t = TransformStamped()

t.header.stamp = node.get_clock().now().to_msg()
t.header.frame_id = 'world'
t.child_frame_id = 'robot_base'
# ... set transform values

br.sendTransform(t)
```

## Package Management

### Package.xml
Defines package metadata and dependencies:

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>My ROS 2 Package</description>
  <maintainer email="me@example.com">My Name</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Ament Build System
ROS 2 uses the ament build system with several flavors:
- `ament_python`: For Python packages
- `ament_cmake`: For CMake-based packages
- `ament_cargo`: For Rust packages

## Common Command Line Tools

### ros2 run
Run a node directly:
```bash
ros2 run package_name executable_name
```

### ros2 topic
Interact with topics:
```bash
ros2 topic list          # List all topics
ros2 topic echo /topic   # Echo messages from topic
ros2 topic info /topic   # Get info about topic
```

### ros2 service
Interact with services:
```bash
ros2 service list        # List all services
ros2 service call /service service_type "{request: value}"
```

### ros2 action
Interact with actions:
```bash
ros2 action list         # List all actions
ros2 action send_goal /action action_type "{goal: value}"
```

### ros2 param
Manage parameters:
```bash
ros2 param list          # List parameters for a node
ros2 param get node_name param_name
ros2 param set node_name param_name value
```

### ros2 node
Manage nodes:
```bash
ros2 node list           # List all nodes
ros2 node info node_name # Get info about a node
```

## Best Practices

### Naming Conventions
- Use lowercase with underscores for node names: `camera_driver`
- Use descriptive names that indicate function
- Avoid special characters in names
- Use consistent naming across packages

### Code Organization
- Keep nodes focused on single responsibilities
- Separate message definitions into dedicated files
- Use parameter files for configuration
- Organize packages by functionality

### Error Handling
- Always check for valid parameters
- Handle connection failures gracefully
- Use proper logging instead of print statements
- Implement proper cleanup in destructors

### Performance
- Use appropriate QoS settings for your use case
- Avoid unnecessary message copying
- Consider using intraprocess communication when appropriate
- Profile your nodes for bottlenecks

## Common Design Patterns

### Publisher-Subscriber Pattern
- Decouples publishers and subscribers
- Enables many-to-many communication
- Asynchronous by nature

### Client-Server Pattern
- Synchronous request-response
- Good for state queries and commands
- Requestor waits for response

### Action Server Pattern
- For long-running operations
- Provides feedback during execution
- Supports goal preemption and cancellation

### Component Architecture
- Combine multiple nodes into single processes
- Reduces inter-process communication overhead
- Enables shared memory communication

## Security Considerations

### ROS 2 Security Features
- DDS Security specification implementation
- Authentication, authorization, and encryption
- Secure communication channels
- Identity management

### Best Practices
- Enable security when deploying in production
- Use appropriate network segmentation
- Validate all incoming data
- Keep dependencies updated

## Testing

### Unit Testing
Use standard testing frameworks with ROS 2:

```python
import unittest
import rclpy
from my_package.my_node import MyNode

class TestMyNode(unittest.TestCase):
    def test_something(self):
        rclpy.init()
        node = MyNode()
        # Test node functionality
        rclpy.shutdown()
```

### Integration Testing
Test multiple nodes working together using launch testing.

### System Testing
Test complete robot systems in simulation and real-world scenarios.

## Debugging

### ROS 2 Tools
- `rqt_graph`: Visualize the computation graph
- `rqt_plot`: Plot numeric values from topics
- `rviz2`: Visualize robot state and sensor data
- `ros2 doctor`: Check ROS 2 installation health

### Logging
Use the built-in logging system:

```python
node.get_logger().info('This is an info message')
node.get_logger().warn('This is a warning')
node.get_logger().error('This is an error')
node.get_logger().fatal('This is fatal')
```

## Summary

This reference covers the essential ROS 2 concepts you need to understand to work effectively with the framework. Use this as a quick lookup when you need to refresh your memory about specific concepts, commands, or best practices.