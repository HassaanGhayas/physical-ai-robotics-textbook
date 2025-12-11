---
title: Sample Components
sidebar_label: Sample Components
description: Demonstrating all custom components for the Physical AI & Humanoid Robotics book
---

# Sample Components

This page demonstrates all the custom components created for the Physical AI & Humanoid Robotics book.

## Hardware Specifications Component

<HardwareSpecs
  name="NVIDIA Jetson AGX Orin"
  category="Processing Unit"
  specs={{
    cpu: "12-core ARM v8.4 64-bit CPU",
    gpu: "512-core NVIDIA Ampere GPU",
    memory: "32GB LPDDR5",
    storage: "64GB eMMC 5.1"
  }}
  cost={{
    price: "$1,099",
    reasoning: "High-performance computing module designed for AI applications"
  }}
  pros={[
    "Powerful GPU for AI inference",
    "Energy efficient",
    "Real-time processing capabilities"
  ]}
  cons={[
    "Expensive",
    "Requires additional cooling"
  ]}
/>

## Technical Diagram Component

<TechnicalDiagrams
  title="Humanoid Robot Architecture"
  description="System architecture diagram showing the main components of a humanoid robot"
  imageUrl="/img/robot-architecture.png"
  caption="High-level architecture of a humanoid robot control system"
  altText="Humanoid robot system architecture diagram"
  interactive={true}
  zoomable={true}
/>

## Code Examples Component

<CodeExamples
  title="ROS2 Control Node"
  description="Sample ROS2 node for controlling robot joints"
  language="python"
  code={`import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.0, 0.0, 0.0]
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()`}
  copyable={true}
  expandable={true}
  maxHeight="300px"
/>

## Book Navigation Component

<BookNavigation
  title="Book Navigation"
  items={[
    {id: "intro", title: "Introduction", url: "/docs/book/introduction", level: 1},
    {id: "hardware", title: "Hardware Requirements", url: "/docs/book/hardware-requirements", level: 1},
    {id: "software", title: "Software Setup", url: "/docs/book/software-setup", level: 1},
    {id: "control", title: "Control Systems", url: "/docs/book/control-systems", level: 1},
    {id: "ai", title: "AI Integration", url: "/docs/book/ai-integration", level: 1},
    {id: "safety", title: "Safety Considerations", url: "/docs/book/safety-considerations", level: 1},
  ]}
  currentId="sample"
  showProgress={true}
  showBookmarks={true}
/>

## Theme Switcher Component

<ThemeSwitcher />