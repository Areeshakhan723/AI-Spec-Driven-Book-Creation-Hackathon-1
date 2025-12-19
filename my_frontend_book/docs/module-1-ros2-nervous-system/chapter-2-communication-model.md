---
title: ROS 2 Communication Model
description: Learn about ROS 2 communication patterns including nodes, topics, services, and rclpy-based agent ↔ controller flow
sidebar_label: ROS 2 Communication Model
sidebar_position: 3
tags: [ros2, communication, nodes, topics, services, rclpy]
authors: [Spec-Kit Plus Team]
keywords: [ROS 2, communication, nodes, topics, services, rclpy, agent, controller]
---

# ROS 2 Communication Model

## Nodes and Processes

In ROS 2, a **node** is an executable that uses the ROS 2 client library. Nodes are the fundamental building blocks of a ROS 2 system and provide structure for organizing computation in a distributed system.

### Definition and Purpose

A node serves as a container for computation that communicates with other nodes through ROS 2's communication infrastructure. Each node:

- Performs specific tasks or functions
- Communicates with other nodes through topics, services, etc.
- Can be written in different programming languages
- Runs as a separate process on the system

### Node Creation and Lifecycle

Nodes have a well-defined lifecycle that includes:

- **Initialization**: Setting up the node and its communication interfaces
- **Configuration**: Configuring parameters and initial settings
- **Activation**: Becoming active and starting to process data
- **Running**: Processing callbacks and performing computations
- **Shutdown**: Cleaning up resources and terminating gracefully

### Client Libraries

ROS 2 supports multiple client libraries for different programming languages:

- **rclcpp**: C++ client library
- **rclpy**: Python client library (most common for beginners)
- **rclnodejs**: JavaScript/TypeScript client library
- **rclrs**: Rust client library

### Process Management

Each node runs as a separate process, which provides:

- **Isolation**: If one node fails, it doesn't crash the entire system
- **Resource Management**: Each node can be allocated specific resources
- **Security**: Nodes can run with different security contexts
- **Distribution**: Nodes can run on different machines

### Naming Conventions

Nodes follow specific naming conventions:

- Names should be descriptive and meaningful
- Use lowercase letters, numbers, and underscores
- Avoid special characters that might cause issues
- Follow the pattern: `node_name` (e.g., `camera_driver`, `navigation_controller`)

## Topics and Message Passing

Topics are named buses over which nodes exchange messages in a publisher-subscriber communication pattern.

### Publisher-Subscriber Pattern

The publisher-subscriber pattern is fundamental to ROS 2:

- **Publishers**: Nodes that send messages to a topic
- **Subscribers**: Nodes that receive messages from a topic
- **Topics**: Named channels for message distribution
- **Asynchronous**: Communication is non-blocking and decoupled

### Topic Communication Characteristics

- **Many-to-Many**: Multiple publishers can send to the same topic
- **Many-to-Many**: Multiple subscribers can receive from the same topic
- **Unidirectional**: Messages flow in one direction (publisher → topic → subscriber)
- **Asynchronous**: Publishers and subscribers don't need to be synchronized

### Message Types and Definitions

Messages have specific types that define their structure:

- **Standard Types**: Built-in message types (strings, numbers, arrays)
- **Custom Types**: User-defined message structures
- **Packages**: Messages organized in packages for reusability
- **Serialization**: Messages are serialized for network transmission

### Synchronization Patterns

Different synchronization approaches are available:

- **Fire-and-Forget**: Publish messages without waiting for acknowledgment
- **Rate-Based**: Publish messages at a specific frequency
- **Event-Driven**: Publish messages in response to specific events
- **Buffering**: Store and process messages in batches

### Practical Examples with rclpy

Here's a simple example of topic communication using rclpy:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and Action Servers

Services provide a request-response communication pattern that is synchronous and bidirectional.

### Service Definition and Implementation

Services are defined with:

- **Request Message**: Data sent from client to server
- **Response Message**: Data sent from server to client
- **Interface Definition**: Service specification file (.srv)
- **Client and Server**: Separate implementations for request and response

### Request-Response Pattern

The service pattern involves:

- **Service Client**: Sends requests and waits for responses
- **Service Server**: Receives requests and sends responses
- **Synchronous**: Client waits for response before continuing
- **One-to-One**: Typically one client communicates with one server

### Action Servers for Long-Running Tasks

For long-running operations, ROS 2 provides action servers:

- **Goals**: Requests for long-running operations
- **Feedback**: Interim results during operation
- **Results**: Final outcome when operation completes
- **Cancelation**: Ability to cancel ongoing operations

### When to Use Services vs Topics

Choose services when:

- You need a response to a specific request
- The operation is relatively quick
- You need guaranteed delivery
- You want synchronous behavior

Choose topics when:

- You need to broadcast information
- The operation is ongoing or continuous
- You don't need immediate responses
- You want asynchronous behavior

### Service Implementation Example

Here's an example of a service using rclpy:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Agent ↔ Controller Flow

The agent ↔ controller communication pattern is crucial for humanoid robotics, where agents make high-level decisions and controllers execute low-level commands.

### Architectural Patterns

The agent-controller architecture typically involves:

- **Agents**: High-level decision-making components
- **Controllers**: Low-level execution components
- **Goals**: High-level objectives from agents to controllers
- **Feedback**: Status information from controllers to agents

### Goal-Oriented Communication

Agents communicate goals to controllers:

- **Commands**: Specific actions to be executed
- **Parameters**: Configuration for the action
- **Constraints**: Limits or requirements for execution
- **Priority**: Importance level of the goal

### Feedback Mechanisms

Controllers provide feedback to agents:

- **Status Updates**: Current execution status
- **Progress Reports**: How much of the task is complete
- **Error Reports**: Issues encountered during execution
- **Sensor Data**: Information from the environment

### Practical Implementation Examples

Here's an example of agent-controller communication using rclpy:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from example_interfaces.action import Fibonacci

class MinimalActionClient(Node):
    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

class MinimalActionServer(Node):
    def __init__(self):
        super().__init__('minimal_action_server')
        self._action_server = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()
    action_client.send_goal(10)

    executor = MultiThreadedExecutor()
    rclpy.spin(action_client, executor=executor)

if __name__ == '__main__':
    main()
```

## Summary

ROS 2 communication patterns provide flexible ways to connect different parts of a robotic system. Nodes provide the basic execution units, topics enable asynchronous data distribution, services provide synchronous request-response interactions, and action servers handle long-running operations with feedback. The agent-controller pattern is particularly important for humanoid robotics, where high-level decision-making needs to coordinate with low-level execution. Understanding these communication patterns is essential for building effective robotic systems with ROS 2.