---
title: ROS 2 Nodes, Topics, and Services
sidebar_position: 2
description: Understanding the communication patterns in ROS 2
---

# ROS 2 Nodes, Topics, and Services

## Nodes

A node is a process that performs computation. Nodes are the fundamental building blocks of ROS 2 applications. In humanoid robotics, nodes might represent:

- Sensor drivers (camera, IMU, LiDAR)
- Control algorithms (motion planning, trajectory generation)
- Perception systems (object detection, SLAM)
- Actuator interfaces (motor controllers)

### Creating a Node

In Python, a node is typically a class that inherits from `rclpy.Node`:

```python
import rclpy
from rclpy.node import Node

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        # Initialize node components here
```

## Topics

Topics enable unidirectional data flow between nodes using a publish/subscribe model:

- Publishers send data to a topic
- Subscribers receive data from a topic
- Multiple publishers/subscribers can use the same topic
- Communication is asynchronous

### Example: Joint State Topic

```python
from sensor_msgs.msg import JointState

def create_publisher(self):
    self.joint_state_publisher = self.create_publisher(
        JointState,
        'joint_states',
        10
    )
```

## Services

Services enable bidirectional request/response communication:

- A client sends a request to a service
- A server processes the request and sends back a response
- Communication is synchronous

### Example: Motion Service

```python
from humanoid_robot_interfaces.srv import MoveJoint

def create_service(self):
    self.move_joint_service = self.create_service(
        MoveJoint,
        'move_joint',
        self.move_joint_callback
    )
```

## Quality of Service (QoS)

ROS 2 provides Quality of Service profiles to control communication behavior:

- Reliability: Best effort or reliable
- Durability: Volatile or transient local
- History: Keep last N or keep all

## Best Practices for Humanoid Robotics

1. **Modularity**: Each function should be in its own node
2. **Efficiency**: Use appropriate QoS settings for real-time control
3. **Safety**: Implement timeouts and error handling
4. **Scalability**: Design for multiple robots in the same network