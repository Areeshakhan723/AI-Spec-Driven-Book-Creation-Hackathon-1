---
title: Basic ROS 2 Node Tutorial
description: Learn how to create a basic ROS 2 node using rclpy
sidebar_label: Basic ROS 2 Node Tutorial
sidebar_position: 5
tags: [ros2, tutorial, rclpy, node, python]
authors: [Spec-Kit Plus Team]
keywords: [ROS 2, tutorial, rclpy, node, python, basic]
---

# Basic ROS 2 Node Tutorial

This tutorial will guide you through creating your first ROS 2 node using Python and the rclpy client library. By the end of this tutorial, you'll understand the basic structure of a ROS 2 node and be able to create simple publishers and subscribers.

## Prerequisites

Before starting this tutorial, make sure you have:

- ROS 2 installed (Humble Hawksbill or later recommended)
- Python 3.8 or later
- Basic Python programming knowledge
- Understanding of ROS 2 concepts (covered in previous chapters)

## Creating a Simple Publisher Node

Let's start by creating a simple publisher node that sends "Hello World" messages.

### Step 1: Set up the basic node structure

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

### Step 2: Understanding the code

Let's break down the key components:

- **Node Inheritance**: Our class inherits from `rclpy.node.Node`
- **Publisher Creation**: `create_publisher()` creates a publisher for String messages
- **Timer**: Creates a timer that calls the callback function periodically
- **Message Creation**: Creates and populates a String message
- **Publishing**: Sends the message to the topic
- **Logging**: Provides feedback on what's happening

### Step 3: Running the publisher

To run this node:

1. Save the code to a file (e.g., `minimal_publisher.py`)
2. Make sure your ROS 2 environment is sourced
3. Run the node: `python3 minimal_publisher.py`

## Creating a Simple Subscriber Node

Now let's create a subscriber that receives and processes the messages from our publisher.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Creating a Service Server and Client

Let's also create a simple service example that adds two integers.

### Service Server

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

### Service Client

```python
import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (int(sys.argv[1]), int(sys.argv[2]), response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Package Structure

For a complete ROS 2 package, you'll need additional files:

### setup.py

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Basic ROS 2 tutorials',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = my_robot_tutorials.minimal_publisher:main',
            'minimal_subscriber = my_robot_tutorials.minimal_subscriber:main',
        ],
    },
)
```

### package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_tutorials</name>
  <version>0.0.0</version>
  <description>Basic ROS 2 tutorials</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Running the Nodes

To run the publisher and subscriber nodes simultaneously:

1. Terminal 1: `ros2 run my_robot_tutorials minimal_publisher`
2. Terminal 2: `ros2 run my_robot_tutorials minimal_subscriber`

You should see the publisher sending messages and the subscriber receiving them.

## Common Issues and Troubleshooting

### Node Names

- Each node must have a unique name on the ROS 2 graph
- If you get naming conflicts, use unique names or namespaces

### Topic Names

- Make sure publisher and subscriber use the same topic name
- Check for typos in topic names

### Permissions

- Make sure your Python files have execute permissions
- Use `chmod +x filename.py` if needed

## Summary

This tutorial covered the basics of creating ROS 2 nodes using Python and rclpy. You learned how to:

- Create publisher and subscriber nodes
- Understand the basic structure of ROS 2 nodes
- Work with different message types
- Create service servers and clients
- Structure a proper ROS 2 package

These fundamentals are essential for building more complex robotic systems. Practice creating different types of nodes and experiment with various message types to solidify your understanding.