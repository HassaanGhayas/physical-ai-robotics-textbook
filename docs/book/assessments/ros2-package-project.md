---
title: ROS 2 Package Project
sidebar_position: 2
description: Developing a complete ROS 2 package for humanoid robot control
---

# ROS 2 Package Development Project

## Overview

This project requires students to develop a complete ROS 2 package for humanoid robot control. The package should demonstrate proficiency in ROS 2 concepts, including nodes, topics, services, actions, parameters, and best practices for robotics software development.

## Project Requirements

### Core Requirements
1. **Package Structure**: Proper ROS 2 package organization following REP-144
2. **Node Implementation**: At least 3 custom nodes with clear responsibilities
3. **Communication**: Use of topics, services, and actions appropriately
4. **Configuration**: Parameter management and launch files
5. **Documentation**: Comprehensive documentation and examples
6. **Testing**: Unit tests and integration tests
7. **Build System**: Proper CMakeLists.txt (for C++) or setup.py (for Python)

### Humanoid-Specific Requirements
- **Humanoid Control Interface**: Nodes for controlling humanoid robot joints
- **Sensor Integration**: Processing of humanoid-specific sensors (IMU, joint encoders, etc.)
- **Safety Systems**: Implementation of safety checks and emergency stops
- **Balance Control**: Basic balance control algorithms for bipedal robots

## Package Structure

### Recommended Directory Structure
```
humanoid_control/
├── CMakeLists.txt or pyproject.toml
├── package.xml
├── README.md
├── LICENSE
├── config/
│   ├── parameters.yaml
│   └── controllers.yaml
├── launch/
│   ├── humanoid_control.launch.py
│   └── test_nodes.launch.py
├── src/
│   ├── humanoid_controller_node.cpp/py
│   ├── sensor_processor_node.cpp/py
│   ├── safety_monitor_node.cpp/py
│   └── utils/
├── include/humanoid_control/ (for C++)
├── test/
│   ├── test_controller.cpp/py
│   ├── test_sensor_processor.cpp/py
│   └── integration_test.cpp/py
├── scripts/
└── docs/
    ├── user_guide.md
    ├── api_reference.md
    └── tutorials/
```

### Package.xml Configuration
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_control</name>
  <version>0.1.0</version>
  <description>ROS 2 package for humanoid robot control</description>
  <maintainer email="student@example.com">Student Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>control_msgs</depend>
  <depend>trajectory_msgs</depend>
  <depend>builtin_interfaces</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_cmake</build_type>  <!-- or ament_python -->
  </export>
</package>
```

## Node Implementation

### 1. Humanoid Controller Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from humanoid_control_msgs.srv import BalanceControl
from humanoid_control_msgs.action import WalkToPosition
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from threading import Lock

class HumanoidControllerNode(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Declare parameters
        self.declare_parameter('robot_description', '')
        self.declare_parameter('control_frequency', 100)
        self.declare_parameter('max_joint_velocity', 1.0)
        self.declare_parameter('balance_control_enabled', True)

        # Store parameters
        self.control_freq = self.get_parameter('control_frequency').value
        self.max_velocity = self.get_parameter('max_joint_velocity').value
        self.balance_enabled = self.get_parameter('balance_control_enabled').value

        # Create publishers
        self.joint_trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Create subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Create service servers
        self.balance_srv = self.create_service(
            BalanceControl,
            'balance_control',
            self.balance_control_callback
        )

        # Create action server
        self.walk_action_server = ActionServer(
            self,
            WalkToPosition,
            'walk_to_position',
            self.execute_walk_goal,
            goal_callback=self.walk_goal_callback,
            cancel_callback=self.walk_cancel_callback
        )

        # Internal state
        self.current_joint_states = {}
        self.control_lock = Lock()
        self.trajectory_executor = TrajectoryExecutor()

        # Control timer
        self.control_timer = self.create_timer(
            1.0/self.control_freq,
            self.control_loop
        )

        self.get_logger().info('Humanoid Controller Node initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        with self.control_lock:
            for name, position in zip(msg.name, msg.position):
                self.current_joint_states[name] = position

    def balance_control_callback(self, request, response):
        """Handle balance control requests"""
        try:
            if request.enable:
                self.enable_balance_control()
                response.success = True
                response.message = "Balance control enabled"
            else:
                self.disable_balance_control()
                response.success = True
                response.message = "Balance control disabled"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def control_loop(self):
        """Main control loop for humanoid control"""
        with self.control_lock:
            # Process safety checks
            if self.should_stop():
                self.emergency_stop()
                return

            # Update balance control if enabled
            if self.balance_enabled:
                self.update_balance_control()

            # Process queued trajectories
            self.trajectory_executor.process_queue()

    def enable_balance_control(self):
        """Enable balance control algorithms"""
        self.balance_enabled = True
        self.get_logger().info("Balance control enabled")

    def disable_balance_control(self):
        """Disable balance control algorithms"""
        self.balance_enabled = False
        self.get_logger().info("Balance control disabled")

    def should_stop(self):
        """Check if emergency stop conditions are met"""
        # Implement safety checks
        # - joint limit violations
        # - balance threshold exceeded
        # - external stop command
        return False

    def emergency_stop(self):
        """Execute emergency stop procedure"""
        self.get_logger().warn("Emergency stop activated!")
        # Send zero velocity commands
        # Disable motors if possible
        # Log the event
```

### 2. Sensor Processor Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Float64MultiArray
from humanoid_control_msgs.msg import BalanceState, JointHealth
import numpy as np
from collections import deque
import tf_transformations

class SensorProcessorNode(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Parameters
        self.declare_parameter('imu_calibration_enabled', True)
        self.declare_parameter('health_check_interval', 1.0)
        self.declare_parameter('balance_threshold', 15.0)  # degrees

        # Store parameters
        self.calibration_enabled = self.get_parameter('imu_calibration_enabled').value
        self.health_interval = self.get_parameter('health_check_interval').value
        self.balance_threshold = self.get_parameter('balance_threshold').value

        # Publishers
        self.balance_pub = self.create_publisher(BalanceState, 'balance_state', 10)
        self.health_pub = self.create_publisher(JointHealth, 'joint_health', 10)
        self.com_pub = self.create_publisher(Vector3, 'center_of_mass', 10)

        # Subscribers
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        # Internal state
        self.imu_data = None
        self.joint_data = None
        self.imu_calibration = np.zeros(3)  # bias offsets
        self.joint_history = deque(maxlen=100)  # for velocity estimation

        # Timers
        self.processing_timer = self.create_timer(0.01, self.process_sensors)  # 100Hz
        self.health_timer = self.create_timer(self.health_interval, self.check_health)

        # Calibration
        if self.calibration_enabled:
            self.calibrate_imu()

    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg

    def joint_callback(self, msg):
        """Process joint state data"""
        self.joint_data = msg
        # Store for history
        self.joint_history.append({
            'stamp': msg.header.stamp,
            'names': msg.name,
            'positions': msg.position,
            'velocities': msg.velocity
        })

    def process_sensors(self):
        """Process all sensor data and publish processed information"""
        if self.imu_data is None or self.joint_data is None:
            return

        # Process IMU data
        balance_state = self.compute_balance_state()
        self.balance_pub.publish(balance_state)

        # Compute center of mass
        com = self.compute_center_of_mass()
        self.com_pub.publish(com)

    def compute_balance_state(self):
        """Compute current balance state from IMU data"""
        # Extract orientation
        orientation = self.imu_data.orientation
        euler_angles = tf_transformations.euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        # Calculate balance metrics
        roll, pitch, yaw = euler_angles
        balance_metric = np.sqrt(roll**2 + pitch**2) * 180.0 / np.pi  # Convert to degrees

        # Create balance state message
        balance_state = BalanceState()
        balance_state.header.stamp = self.get_clock().now().to_msg()
        balance_state.roll = roll
        balance_state.pitch = pitch
        balance_state.yaw = yaw
        balance_state.balance_metric = balance_metric
        balance_state.is_stable = balance_metric < self.balance_threshold

        return balance_state

    def compute_center_of_mass(self):
        """Estimate center of mass based on joint positions and masses"""
        # This is a simplified example - real implementation would use URDF
        if not self.joint_data:
            return Vector3(x=0.0, y=0.0, z=0.0)

        # Placeholder: return estimated COM based on joint positions
        # Real implementation would use forward kinematics and link masses
        com = Vector3()
        com.x = 0.0  # Placeholder
        com.y = 0.0  # Placeholder
        com.z = 0.8  # Approximate height for humanoid

        return com

    def check_health(self):
        """Check joint health and publish status"""
        if not self.joint_data:
            return

        health_msg = JointHealth()
        health_msg.header.stamp = self.get_clock().now().to_msg()

        for i, joint_name in enumerate(self.joint_data.name):
            # Check position limits, temperature, etc.
            position_ok = self.check_joint_position(joint_name, self.joint_data.position[i])
            temp_ok = self.check_joint_temperature(joint_name)  # Placeholder
            velocity_ok = self.check_joint_velocity(joint_name, self.joint_data.velocity[i])

            joint_health = JointHealthStatus()
            joint_health.joint_name = joint_name
            joint_health.position_ok = position_ok
            joint_health.temperature_ok = temp_ok
            joint_health.velocity_ok = velocity_ok
            joint_health.health_score = self.calculate_health_score(
                position_ok, temp_ok, velocity_ok
            )

            health_msg.joint_statuses.append(joint_health)

        self.health_pub.publish(health_msg)

    def calibrate_imu(self):
        """Calibrate IMU sensors"""
        self.get_logger().info("Starting IMU calibration...")
        # Collect data for calibration
        # Calculate bias offsets
        # Store calibration parameters
        self.get_logger().info("IMU calibration completed")
```

### 3. Safety Monitor Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from humanoid_control_msgs.msg import BalanceState, JointHealth
from builtin_interfaces.msg import Time
from threading import Lock

class SafetyMonitorNode(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Parameters
        self.declare_parameter('emergency_stop_enabled', True)
        self.declare_parameter('balance_threshold', 20.0)
        self.declare_parameter('temperature_threshold', 75.0)
        self.declare_parameter('position_tolerance', 0.1)

        # Store parameters
        self.emergency_enabled = self.get_parameter('emergency_stop_enabled').value
        self.balance_thresh = self.get_parameter('balance_threshold').value
        self.temp_thresh = self.get_parameter('temperature_threshold').value
        self.pos_tol = self.get_parameter('position_tolerance').value

        # Publishers
        self.emergency_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        self.safety_status_pub = self.create_publisher(String, 'safety_status', 10)

        # Subscribers
        self.balance_sub = self.create_subscription(
            BalanceState, 'balance_state', self.balance_callback, 10
        )
        self.joint_health_sub = self.create_subscription(
            JointHealth, 'joint_health', self.health_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        # Internal state
        self.current_balance = None
        self.joint_health_status = {}
        self.safety_lock = Lock()
        self.safety_engaged = False

        # Safety monitoring timer
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Safety Monitor Node initialized')

    def balance_callback(self, msg):
        """Process balance state messages"""
        self.current_balance = msg

    def health_callback(self, msg):
        """Process joint health messages"""
        for status in msg.joint_statuses:
            self.joint_health_status[status.joint_name] = status

    def joint_state_callback(self, msg):
        """Process joint state messages for position/velocity checks"""
        # Store for safety checks
        pass

    def safety_check(self):
        """Main safety monitoring function"""
        with self.safety_lock:
            issues = []

            # Check balance
            if self.current_balance:
                if self.current_balance.balance_metric > self.balance_thresh:
                    issues.append(f"Balance threshold exceeded: {self.current_balance.balance_metric:.2f}° > {self.balance_thresh}°")

            # Check joint health
            for joint_name, health_status in self.joint_health_status.items():
                if not health_status.position_ok:
                    issues.append(f"Joint {joint_name} position limit exceeded")
                if not health_status.temperature_ok:
                    issues.append(f"Joint {joint_name} temperature threshold exceeded")
                if not health_status.velocity_ok:
                    issues.append(f"Joint {joint_name} velocity limit exceeded")

            # Publish safety status
            status_msg = String()
            if issues:
                status_msg.data = "ISSUES: " + "; ".join(issues)
                self.get_logger().warn(status_msg.data)

                # Check if emergency stop is needed
                if self.emergency_enabled and self.should_emergency_stop(issues):
                    self.trigger_emergency_stop()
                    status_msg.data = "EMERGENCY STOP ACTIVATED: " + status_msg.data
            else:
                status_msg.data = "SAFE: All systems nominal"
                if self.safety_engaged:
                    self.release_emergency_stop()

            self.safety_status_pub.publish(status_msg)

    def should_emergency_stop(self, issues):
        """Determine if emergency stop should be triggered"""
        critical_issues = [
            issue for issue in issues
            if "Balance threshold" in issue or "temperature threshold" in issue
        ]
        return len(critical_issues) > 0

    def trigger_emergency_stop(self):
        """Trigger emergency stop"""
        if not self.safety_engaged:
            self.safety_engaged = True
            self.get_logger().fatal("EMERGENCY STOP TRIGGERED!")

            # Publish emergency stop message
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_pub.publish(stop_msg)

    def release_emergency_stop(self):
        """Release emergency stop"""
        if self.safety_engaged:
            self.safety_engaged = False
            self.get_logger().info("Emergency stop released")

            # Publish release message
            release_msg = Bool()
            release_msg.data = False
            self.emergency_pub.publish(release_msg)
```

## Launch Files

### Main Launch File
```python
# launch/humanoid_control.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the nodes'
    )

    config_dir = os.path.join(get_package_share_directory('humanoid_control'), 'config')

    return LaunchDescription([
        namespace_arg,

        # Humanoid Controller Node
        Node(
            package='humanoid_control',
            executable='humanoid_controller',
            name='humanoid_controller',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                os.path.join(config_dir, 'parameters.yaml'),
                {'robot_description': ''},
                {'control_frequency': 100},
                {'max_joint_velocity': 1.0},
                {'balance_control_enabled': True}
            ],
            output='screen'
        ),

        # Sensor Processor Node
        Node(
            package='humanoid_control',
            executable='sensor_processor',
            name='sensor_processor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                os.path.join(config_dir, 'parameters.yaml'),
                {'imu_calibration_enabled': True},
                {'health_check_interval': 1.0},
                {'balance_threshold': 15.0}
            ],
            output='screen'
        ),

        # Safety Monitor Node
        Node(
            package='humanoid_control',
            executable='safety_monitor',
            name='safety_monitor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[
                os.path.join(config_dir, 'parameters.yaml'),
                {'emergency_stop_enabled': True},
                {'balance_threshold': 20.0},
                {'temperature_threshold': 75.0},
                {'position_tolerance': 0.1}
            ],
            output='screen'
        )
    ])
```

## Testing Framework

### Unit Tests
```python
# test/test_controller.py
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from humanoid_control.humanoid_controller_node import HumanoidControllerNode

class TestHumanoidController(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = HumanoidControllerNode()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def tearDown(self):
        self.node.destroy_node()

    def test_node_initialization(self):
        """Test that the node initializes properly"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.count, 0)

    def test_parameters(self):
        """Test that parameters are declared and accessible"""
        self.assertTrue(self.node.has_parameter('control_frequency'))
        self.assertTrue(self.node.has_parameter('max_joint_velocity'))
        self.assertTrue(self.node.has_parameter('balance_control_enabled'))

    def test_publishers_created(self):
        """Test that required publishers are created"""
        # Check that publishers exist
        pub_names = [pub.topic_name for pub in self.node.publishers]
        self.assertIn('/joint_trajectory_controller/joint_trajectory', pub_names)

if __name__ == '__main__':
    unittest.main()
```

## Documentation

### User Guide
The user guide should include:
- Installation instructions
- Configuration options
- Usage examples
- Troubleshooting tips
- API reference

### Tutorials
- Basic setup and configuration
- Running the controller nodes
- Sending commands to the humanoid
- Monitoring safety systems
- Customizing parameters

## Build System Configuration

### CMakeLists.txt (for C++ implementation)
```cmake
cmake_minimum_required(VERSION 3.8)
project(humanoid_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(humanoid_control_msgs REQUIRED)  # Custom message package

# Include directories
include_directories(include)

# Add executables
add_executable(humanoid_controller src/humanoid_controller_node.cpp)
add_executable(sensor_processor src/sensor_processor_node.cpp)
add_executable(safety_monitor src/safety_monitor_node.cpp)

# Link libraries
ament_target_dependencies(humanoid_controller
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "control_msgs"
  "trajectory_msgs"
  "builtin_interfaces"
  "humanoid_control_msgs"
)

ament_target_dependencies(sensor_processor
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
  "humanoid_control_msgs"
)

ament_target_dependencies(safety_monitor
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "builtin_interfaces"
  "humanoid_control_msgs"
)

# Install executables
install(TARGETS
  humanoid_controller
  sensor_processor
  safety_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```

## Evaluation Criteria

### Technical Requirements (70%)
- **Code Quality**: Well-structured, commented, following ROS 2 best practices
- **Functionality**: All required features work correctly
- **Integration**: Proper communication between nodes
- **Safety**: Robust safety systems implemented
- **Documentation**: Comprehensive and clear

### Innovation and Complexity (20%)
- **Advanced Features**: Beyond basic requirements
- **Problem Solving**: Creative solutions to challenges
- **Performance**: Efficient algorithms and resource usage

### Testing and Validation (10%)
- **Test Coverage**: Adequate unit and integration tests
- **Validation**: Proper verification of functionality
- **Debugging**: Clear error handling and logging

## Submission Requirements

### Required Files
1. Complete package source code
2. Launch files for different configurations
3. Configuration files
4. Unit and integration tests
5. Documentation files
6. README with build and run instructions
7. Demo video showing functionality (optional but recommended)

### Assessment Rubric
- **Functionality**: Does the package work as specified?
- **Code Quality**: Is the code well-written and maintainable?
- **ROS 2 Compliance**: Does it follow ROS 2 conventions?
- **Safety**: Are safety considerations properly addressed?
- **Documentation**: Is the package well-documented?
- **Testing**: Are there adequate tests?

This project provides comprehensive experience in ROS 2 package development for humanoid robotics, covering all essential aspects from basic communication to advanced safety systems.