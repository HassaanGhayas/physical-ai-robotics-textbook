---
title: Gazebo Implementation Project
sidebar_position: 3
description: Implementing humanoid robot simulation in Gazebo with physics, sensors, and control
---

# Gazebo Implementation Project

## Overview

This project requires students to implement a complete humanoid robot simulation in Gazebo, including accurate physics modeling, sensor simulation, and control systems. Students will learn to create realistic simulation environments that closely match real-world robot behavior.

## Project Requirements

### Core Requirements
1. **Humanoid Robot Model**: Complete URDF model with realistic joint limits and dynamics
2. **Physics Simulation**: Accurate physics modeling with proper mass, inertia, and friction properties
3. **Sensor Integration**: Implementation of multiple sensor types (IMU, cameras, LiDAR, force/torque)
4. **Control Systems**: Integration with ROS 2 control frameworks
5. **Environment Creation**: Custom environments for testing humanoid capabilities
6. **Validation**: Comparison between simulated and expected behavior

### Humanoid-Specific Requirements
- **Bipedal Locomotion**: Stable walking and balance simulation
- **Dynamic Balance**: Center of mass and zero-moment point (ZMP) considerations
- **Multi-contact Points**: Feet, hands, and other contact surfaces
- **Stability Analysis**: Tools for analyzing balance and stability

## Project Structure

### Directory Structure
```
gazebo_humanoid_simulation/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── controllers.yaml
│   ├── gazebo_params.yaml
│   └── robot_properties.yaml
├── launch/
│   ├── robot_spawn.launch.py
│   ├── simulation_world.launch.py
│   └── test_scenarios.launch.py
├── models/
│   ├── humanoid_robot/
│   │   ├── model.urdf.xacro
│   │   ├── materials/
│   │   ├── meshes/
│   │   └── sensors/
│   └── environments/
│       ├── indoor/
│       ├── outdoor/
│       └── testing/
├── worlds/
│   ├── simple_room.world
│   ├── complex_environment.world
│   └── testing_scenarios.world
├── src/
│   ├── gazebo_plugins/
│   │   ├── balance_controller_plugin.cpp
│   │   ├── sensor_interface_plugin.cpp
│   │   └── contact_manager_plugin.cpp
│   ├── control_nodes/
│   │   ├── balance_controller_node.cpp
│   │   ├── walk_generator_node.cpp
│   │   └── sensor_processor_node.cpp
│   └── test_nodes/
│       ├── stability_test_node.cpp
│       ├── balance_analysis_node.cpp
│       └── sensor_validation_node.cpp
├── scripts/
│   ├── generate_urdf.py
│   ├── validate_dynamics.py
│   └── run_tests.sh
└── test/
    ├── test_balance_control.cpp
    ├── test_sensor_accuracy.cpp
    └── integration_tests.py
```

## URDF/XACRO Model Development

### Complete Humanoid URDF
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_robot">

  <!-- Constants -->
  <xacro:property name="PI" value="3.1415926535897931"/>
  <xacro:property name="mass_torso" value="10.0"/>
  <xacro:property name="mass_head" value="2.0"/>
  <xacro:property name="mass_arm" value="1.5"/>
  <xacro:property name="mass_leg" value="3.0"/>
  <xacro:property name="density" value="1000"/>

  <!-- Materials -->
  <material name="black">
    <color rgba="0.0 0.0 0.0 1.0"/>
  </material>
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="green">
    <color rgba="0.0 0.8 0.0 1.0"/>
  </material>
  <material name="grey">
    <color rgba="0.5 0.5 0.5 1.0"/>
  </material>
  <material name="orange">
    <color rgba="1.0 0.4235294117647059 0.0392156862745098 1.0"/>
  </material>
  <material name="brown">
    <color rgba="0.8705882352941177 0.8117647058823529 0.7647058823529411 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="0.0001"/>
      <inertia ixx="0.0001" ixy="0.0" ixz="0.0" iyy="0.0001" iyz="0.0" izz="0.0001"/>
    </inertial>
  </link>

  <!-- Torso -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.6" radius="0.15"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.6" radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.3" rpy="0 0 0"/>
      <mass value="${mass_torso}"/>
      <inertia
        ixx="${mass_torso * (3*0.15*0.15 + 0.6*0.6)/12}"
        ixy="0" ixz="0"
        iyy="${mass_torso * (3*0.15*0.15 + 0.6*0.6)/12}"
        iyz="0"
        izz="${mass_torso * 0.15*0.15 / 2}"/>
    </inertial>
  </link>

  <!-- Head -->
  <joint name="torso_head_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.65" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="100" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="head">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="${mass_head}"/>
      <inertia
        ixx="${2*mass_head*0.15*0.15/5}"
        ixy="0" ixz="0"
        iyy="${2*mass_head*0.15*0.15/5}"
        iyz="0"
        izz="${2*mass_head*0.15*0.15/5}"/>
    </inertial>
  </link>

  <!-- Left Arm -->
  <joint name="torso_left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="50" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="${mass_arm}"/>
      <inertia
        ixx="${mass_arm * (3*0.05*0.05 + 0.3*0.3)/12}"
        ixy="0" ixz="0"
        iyy="${mass_arm * (3*0.05*0.05 + 0.3*0.3)/12}"
        iyz="0"
        izz="${mass_arm * 0.05*0.05 / 2}"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="30" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.04"/>
      </geometry>
      <material name="orange"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="${mass_arm*0.7}"/>
      <inertia
        ixx="${mass_arm*0.7 * (3*0.04*0.04 + 0.3*0.3)/12}"
        ixy="0" ixz="0"
        iyy="${mass_arm*0.7 * (3*0.04*0.04 + 0.3*0.3)/12}"
        iyz="0"
        izz="${mass_arm*0.7 * 0.04*0.04 / 2}"/>
    </inertial>
  </link>

  <!-- Left Leg -->
  <joint name="torso_left_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_thigh"/>
    <origin xyz="0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="100" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="${mass_leg}"/>
      <inertia
        ixx="${mass_leg * (3*0.08*0.08 + 0.5*0.5)/12}"
        ixy="0" ixz="0"
        iyy="${mass_leg * (3*0.08*0.08 + 0.5*0.5)/12}"
        iyz="0"
        izz="${mass_leg * 0.08*0.08 / 2}"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="0" effort="80" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="left_shin">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="${mass_leg*0.9}"/>
      <inertia
        ixx="${mass_leg*0.9 * (3*0.07*0.07 + 0.5*0.5)/12}"
        ixy="0" ixz="0"
        iyy="${mass_leg*0.9 * (3*0.07*0.07 + 0.5*0.5)/12}"
        iyz="0"
        izz="${mass_leg*0.9 * 0.07*0.07 / 2}"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/4}" upper="${PI/4}" effort="50" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia
        ixx="0.01" ixy="0" ixz="0"
        iyy="0.01" iyz="0"
        izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Leg (mirrored) -->
  <joint name="torso_right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_thigh"/>
    <origin xyz="-0.05 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="${PI/2}" effort="100" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.08"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="${mass_leg}"/>
      <inertia
        ixx="${mass_leg * (3*0.08*0.08 + 0.5*0.5)/12}"
        ixy="0" ixz="0"
        iyy="${mass_leg * (3*0.08*0.08 + 0.5*0.5)/12}"
        iyz="0"
        izz="${mass_leg * 0.08*0.08 / 2}"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/2}" upper="0" effort="80" velocity="1.0"/>
    <dynamics damping="0.2" friction="0.0"/>
  </joint>

  <link name="right_shin">
    <visual>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.07"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <geometry>
        <capsule length="0.5" radius="0.07"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.25" rpy="0 0 0"/>
      <mass value="${mass_leg*0.9}"/>
      <inertia
        ixx="${mass_leg*0.9 * (3*0.07*0.07 + 0.5*0.5)/12}"
        ixy="0" ixz="0"
        iyy="${mass_leg*0.9 * (3*0.07*0.07 + 0.5*0.5)/12}"
        iyz="0"
        izz="${mass_leg*0.9 * 0.07*0.07 / 2}"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.5" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="${-PI/4}" upper="${PI/4}" effort="50" velocity="1.0"/>
    <dynamics damping="0.1" friction="0.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0 -0.05" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia
        ixx="0.01" ixy="0" ixz="0"
        iyy="0.01" iyz="0"
        izz="0.01"/>
    </inertial>
  </link>

  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/humanoid</robotNamespace>
      <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
  </gazebo>

  <!-- Sensors -->
  <xacro:include filename="$(find gazebo_humanoid_simulation)/urdf/imu.xacro"/>
  <xacro:include filename="$(find gazebo_humanoid_simulation)/urdf/camera.xacro"/>
  <xacro:include filename="$(find gazebo_humanoid_simulation)/urdf/lidar.xacro"/>

</robot>
```

## Gazebo Plugins Development

### Balance Control Plugin
```cpp
// include/gazebo_humanoid_simulation/balance_control_plugin.h
#ifndef GAZEBO_HUMANOID_BALANCE_CONTROL_PLUGIN_H
#define GAZEBO_HUMANOID_BALANCE_CONTROL_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Events.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>

namespace gazebo {

class BalanceControlPlugin : public ModelPlugin {
public:
  BalanceControlPlugin();
  virtual ~BalanceControlPlugin();

  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();

private:
  void UpdateChild();
  void OnContact(const boost::shared_ptr<const gazebo::msgs::Contacts> &_msg);
  ignition::math::Vector3<double> ComputeZMP();
  void ComputeBalanceControl();
  void PublishBalanceData();

private:
  physics::ModelPtr model;
  physics::PhysicsEnginePtr physics;
  event::ConnectionPtr update_connection_;

  // Joint controllers
  std::vector<physics::JointPtr> joints;
  std::vector<std::string> joint_names;

  // Balance control parameters
  double balance_gain;
  double com_height;
  double zmp_threshold;
  bool balance_enabled;

  // Sensors
  physics::LinkPtr torso_link;
  physics::LinkPtr left_foot_link;
  physics::LinkPtr right_foot_link;

  // ROS interface
  ros::NodeHandle* rosnode_;
  ros::Publisher balance_pub_;
  ros::Subscriber balance_cmd_sub_;

  // Contact information
  std::vector<std::string> contacts;

  // Timing
  ros::Time last_update_time;
};

}
#endif
```

```cpp
// src/balance_control_plugin.cpp
#include <gazebo_humanoid_simulation/balance_control_plugin.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <thread>

namespace gazebo {

BalanceControlPlugin::BalanceControlPlugin() : balance_gain(1.0),
                                               com_height(0.8),
                                               zmp_threshold(0.05),
                                               balance_enabled(true) {
}

void BalanceControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
  this->model = _model;
  this->physics = _model->GetWorld()->Physics();

  // Get links
  this->torso_link = _model->GetLink("torso");
  this->left_foot_link = _model->GetLink("left_foot");
  this->right_foot_link = _model->GetLink("right_foot");

  // Get joints
  std::vector<std::string> joint_names_list = {
    "torso_left_hip_joint", "left_knee_joint", "left_ankle_joint",
    "torso_right_hip_joint", "right_knee_joint", "right_ankle_joint",
    "torso_left_shoulder_joint", "left_elbow_joint",
    "torso_right_shoulder_joint", "right_elbow_joint",
    "torso_head_joint"
  };

  for (const auto& name : joint_names_list) {
    physics::JointPtr joint = _model->GetJoint(name);
    if (joint) {
      joints.push_back(joint);
      joint_names.push_back(name);
    }
  }

  // Load parameters
  if (_sdf->HasElement("balance_gain")) {
    balance_gain = _sdf->Get<double>("balance_gain");
  }
  if (_sdf->HasElement("zmp_threshold")) {
    zmp_threshold = _sdf->Get<double>("zmp_threshold");
  }

  // Initialize ROS
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_balance_control", ros::init_options::NoSigintHandler);
  }

  this->rosnode_ = new ros::NodeHandle("balance_control");

  // Publishers and subscribers
  this->balance_pub_ = this->rosnode_->advertise<geometry_msgs::WrenchStamped>(
    "balance_state", 10);

  // Connect to world update event
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&BalanceControlPlugin::UpdateChild, this));

  gzdbg << "Balance Control Plugin loaded for model " << _model->GetName() << std::endl;
}

void BalanceControlPlugin::UpdateChild() {
  if (!balance_enabled) return;

  // Compute current ZMP
  ignition::math::Vector3<double> zmp = ComputeZMP();

  // Compute balance correction
  ComputeBalanceControl();

  // Publish balance data
  PublishBalanceData();
}

ignition::math::Vector3<double> BalanceControlPlugin::ComputeZMP() {
  // Compute Zero-Moment Point (ZMP)
  // This is a simplified implementation
  // In reality, this would involve more complex physics calculations

  if (!torso_link) return ignition::math::Vector3<double>(0, 0, 0);

  // Get CoM position
  ignition::math::Vector3<double> com = torso_link->WorldInertialPose().Pos();

  // Get contact forces (simplified)
  ignition::math::Vector3<double> total_force(0, 0, 0);
  ignition::math::Vector3<double> total_moment(0, 0, 0);

  // Calculate ZMP based on contact points and forces
  // ZMP = CoM - (g/z_com) * (CoM_acceleration)
  // For simulation, we'll use a simplified approach

  // Calculate based on foot contacts
  ignition::math::Vector3<double> left_foot_pos = left_foot_link->WorldInertialPose().Pos();
  ignition::math::Vector3<double> right_foot_pos = right_foot_link->WorldInertialPose().Pos();

  // Average of foot positions as approximate ZMP
  ignition::math::Vector3<double> avg_foot_pos =
    (left_foot_pos + right_foot_pos) / 2.0;

  // Return ZMP relative to CoM
  return avg_foot_pos - ignition::math::Vector3<double>(com.X(), com.Y(), 0);
}

void BalanceControlPlugin::ComputeBalanceControl() {
  ignition::math::Vector3<double> zmp = ComputeZMP();

  // Check if ZMP is within support polygon
  double zmp_distance = sqrt(zmp.X()*zmp.X() + zmp.Y()*zmp.Y());

  if (zmp_distance > zmp_threshold) {
    // Apply corrective torques to joints to restore balance
    // This is a simplified control approach

    // Calculate correction based on ZMP deviation
    double correction_x = -balance_gain * zmp.X();
    double correction_y = -balance_gain * zmp.Y();

    // Apply to hip and ankle joints for balance correction
    for (size_t i = 0; i < joints.size(); ++i) {
      physics::JointPtr joint = joints[i];

      // Apply corrective torques based on joint type and position
      if (joint->GetType() == physics::Joint::HINGE_JOINT) {
        double torque = 0.0;

        // Hip joints for CoM adjustment
        if (joint->GetName().find("hip") != std::string::npos) {
          if (joint->GetName().find("left") != std::string::npos) {
            torque = correction_y;
          } else {
            torque = -correction_y;
          }
        }
        // Ankle joints for fine balance
        else if (joint->GetName().find("ankle") != std::string::npos) {
          if (joint->GetName().find("left") != std::string::npos) {
            torque = correction_x;
          } else {
            torque = -correction_x;
          }
        }

        joint->SetForce(0, torque);
      }
    }
  }
}

void BalanceControlPlugin::PublishBalanceData() {
  geometry_msgs::WrenchStamped balance_msg;
  balance_msg.header.stamp = ros::Time::now();
  balance_msg.header.frame_id = "world";

  ignition::math::Vector3<double> zmp = ComputeZMP();
  balance_msg.wrench.force.x = zmp.X();
  balance_msg.wrench.force.y = zmp.Y();
  balance_msg.wrench.force.z = zmp.Z();

  // Calculate balance metric
  double balance_metric = sqrt(zmp.X()*zmp.X() + zmp.Y()*zmp.Y());
  balance_msg.wrench.torque.z = balance_metric;

  this->balance_pub_.publish(balance_msg);
}

void BalanceControlPlugin::Reset() {
  // Reset balance control state
  balance_enabled = true;
}

BalanceControlPlugin::~BalanceControlPlugin() {
  // Disconnect from events
  event::Events::DisconnectWorldUpdateBegin(this->update_connection_);

  // Cleanup ROS
  if (this->rosnode_) {
    delete this->rosnode_;
    this->rosnode_ = NULL;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(BalanceControlPlugin)

}
```

## Environment Creation

### Custom World File
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="humanoid_test_world">
    <!-- Include Gazebo environment -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Custom environment elements -->
    <light name="ambient_light" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>10</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.3 0.3 -1</direction>
    </light>

    <!-- Indoor environment -->
    <model name="room_walls">
      <static>true</static>
      <link name="wall_link">
        <visual name="wall_visual">
          <geometry>
            <box>
              <size>10 0.2 3</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
        <collision name="wall_collision">
          <geometry>
            <box>
              <size>10 0.2 3</size>
            </box>
          </geometry>
        </collision>
      </link>
      <pose>0 -5 1.5 0 0 0</pose>
    </model>

    <!-- Obstacles for testing -->
    <model name="obstacle_box">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="box_link">
        <inertial>
          <mass>5.0</mass>
          <inertia>
            <ixx>0.1</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.1</iyy>
            <iyz>0</iyz>
            <izz>0.1</izz>
          </inertia>
        </inertial>
        <visual name="box_visual">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.6 0.4 0.2 1</diffuse>
          </material>
        </visual>
        <collision name="box_collision">
          <geometry>
            <box>
              <size>0.5 0.5 1.0</size>
            </box>
          </geometry>
        </collision>
      </link>
    </model>

    <!-- Sloped surface for balance testing -->
    <model name="sloped_surface">
      <static>true</static>
      <link name="slope_link">
        <visual name="slope_visual">
          <geometry>
            <mesh>
              <uri>file://meshes/slope.dae</uri>
            </mesh>
          </geometry>
        </visual>
        <collision name="slope_collision">
          <geometry>
            <mesh>
              <uri>file://meshes/slope.dae</uri>
            </mesh>
          </geometry>
        </collision>
      </link>
      <pose>5 0 0 0 0.2 0</pose>
    </model>

    <!-- Humanoid robot spawn -->
    <include>
      <name>humanoid_robot</name>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Physics parameters -->
    <physics name="ode" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
  </world>
</sdf>
```

## Control System Integration

### ROS 2 Controllers Configuration
```yaml
# config/controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    # Joint trajectory controller
    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # Balance controller
    balance_controller:
      type: humanoid_control/BalanceController

    # Force/torque controller
    impedance_controller:
      type: humanoid_control/ImpedanceController

# Joint trajectory controller configuration
joint_trajectory_controller:
  ros__parameters:
    joints:
      - torso_left_hip_joint
      - left_knee_joint
      - left_ankle_joint
      - torso_right_hip_joint
      - right_knee_joint
      - right_ankle_joint
      - torso_left_shoulder_joint
      - left_elbow_joint
      - torso_right_shoulder_joint
      - right_elbow_joint
      - torso_head_joint

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.0

# Balance controller configuration
balance_controller:
  ros__parameters:
    kp_com: [100.0, 100.0, 0.0]  # Proportional gains for CoM control
    kd_com: [10.0, 10.0, 0.0]    # Derivative gains for CoM control
    kp_foot: [50.0, 50.0, 100.0] # Proportional gains for foot control
    kd_foot: [5.0, 5.0, 10.0]    # Derivative gains for foot control
    zmp_threshold: 0.05          # Maximum allowable ZMP deviation
    com_height_ref: 0.85         # Reference CoM height
    control_frequency: 500       # Balance control frequency

# Impedance controller configuration
impedance_controller:
  ros__parameters:
    stiffness:
      trans_x: 1000.0
      trans_y: 1000.0
      trans_z: 3000.0
      rot_x: 300.0
      rot_y: 300.0
      rot_z: 300.0

    damping_ratio: 1.0
    control_mode: impedance
```

## Sensor Integration

### IMU Configuration
```xml
<!-- urdf/imu.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="imu_macro" params="name parent_link *origin">
    <joint name="${name}_joint" type="fixed">
      <xacro:insert_block name="origin"/>
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
    </joint>

    <link name="${name}_link">
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
    </link>

    <gazebo reference="${name}_link">
      <sensor name="${name}_sensor" type="imu">
        <always_on>true</always_on>
        <update_rate>100</update_rate>
        <visualize>false</visualize>
        <topic>__default_topic__</topic>
        <pose>0 0 0 0 0 0</pose>
        <imu>
          <noise>
            <type>gaussian</type>
            <rate>
              <mean>0.0</mean>
              <stddev>2e-4</stddev>
              <bias_mean>0.0000075</bias_mean>
              <bias_stddev>0.0000008</bias_stddev>
            </rate>
            <accel>
              <mean>0.0</mean>
              <stddev>1.7e-2</stddev>
              <bias_mean>0.0</bias_mean>
              <bias_stddev>0.005</bias_stddev>
            </accel>
          </noise>
        </imu>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
```

### Camera Configuration
```xml
<!-- urdf/camera.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="camera_macro" params="name parent_link *origin">
    <joint name="${name}_joint" type="fixed">
      <xacro:insert_block name="origin"/>
      <parent link="${parent_link}"/>
      <child link="${name}_link"/>
    </joint>

    <link name="${name}_link">
      <inertial>
        <mass value="0.01"/>
        <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
      </inertial>
      <visual>
        <geometry>
          <box size="0.02 0.04 0.02"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.02 0.04 0.02"/>
        </geometry>
      </collision>
    </link>

    <gazebo reference="${name}_link">
      <sensor name="${name}_camera" type="camera">
        <update_rate>30</update_rate>
        <camera name="head">
          <horizontal_fov>1.3962634</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
        <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
          <frame_name>${name}_optical_frame</frame_name>
          <min_depth>0.1</min_depth>
          <max_depth>100</max_depth>
        </plugin>
      </sensor>
    </gazebo>
  </xacro:macro>
</robot>
```

## Testing and Validation

### Test Scenarios
```python
# test/scenario_tests.py
import unittest
import rospy
import rostest
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float64
import time

class TestBalanceControl(unittest.TestCase):
    def setUp(self):
        rospy.init_node('balance_test_node', anonymous=True)
        self.balance_sub = rospy.Subscriber('/balance_state', WrenchStamped, self.balance_callback)
        self.balance_msg = None
        self.received = False

    def balance_callback(self, msg):
        self.balance_msg = msg
        self.received = True

    def test_initial_balance(self):
        """Test that robot starts in balanced state"""
        # Wait for initial balance message
        timeout = rospy.Duration.from_sec(5.0)
        start_time = rospy.Time.now()

        while not self.received and (rospy.Time.now() - start_time) < timeout:
            rospy.sleep(0.1)

        self.assertTrue(self.received, "Balance message not received")
        self.assertIsNotNone(self.balance_msg, "Balance message is None")

        # Check initial balance is within acceptable range
        balance_metric = self.balance_msg.wrench.torque.z
        self.assertLess(balance_metric, 0.1, "Initial balance exceeds threshold")

    def test_disturbance_response(self):
        """Test robot response to external disturbance"""
        # Apply external force to robot
        # This would require a service call or publisher to apply force
        # For now, we'll just verify the test structure

        # Wait for balance to stabilize after disturbance
        time.sleep(2.0)

        # Check that balance is restored
        if self.balance_msg:
            balance_after_disturbance = self.balance_msg.wrench.torque.z
            self.assertLess(balance_after_disturbance, 0.2, "Balance not restored after disturbance")

class TestLocomotion(unittest.TestCase):
    def setUp(self):
        rospy.init_node('locomotion_test_node', anonymous=True)
        self.step_count = 0
        self.step_sub = rospy.Subscriber('/step_events', Float64, self.step_callback)

    def step_callback(self, msg):
        self.step_count += 1

    def test_forward_locomotion(self):
        """Test basic forward walking"""
        initial_steps = self.step_count

        # Command robot to walk forward
        # This would involve sending commands to the walk controller

        # Wait for walking to occur
        time.sleep(5.0)

        # Verify steps were taken
        steps_taken = self.step_count - initial_steps
        self.assertGreater(steps_taken, 0, "No steps detected during forward walking")

if __name__ == '__main__':
    rostest.rosrun('gazebo_humanoid_simulation', 'test_balance_control', TestBalanceControl)
    rostest.rosrun('gazebo_humanoid_simulation', 'test_locomotion', TestLocomotion)
```

## Launch Files

### Main Simulation Launch
```python
# launch/humanoid_simulation.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_test_world',
        description='Choose one of the world files from GAZEBO_MODEL_PATH'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('gazebo_humanoid_simulation'),
                'worlds',
                LaunchConfiguration('world')
            ])
        }.items()
    )

    # Robot spawn node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', LaunchConfiguration('robot_name'),
            '-x', '0',
            '-y', '0',
            '-z', '1.0'
        ],
        output='screen'
    )

    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('gazebo_humanoid_simulation'),
                'config',
                'controllers.yaml'
            ])
        ],
        output='screen'
    )

    # Balance controller
    balance_controller = Node(
        package='gazebo_humanoid_simulation',
        executable='balance_controller_node',
        name='balance_controller',
        output='screen'
    )

    # Sensor processor
    sensor_processor = Node(
        package='gazebo_humanoid_simulation',
        executable='sensor_processor_node',
        name='sensor_processor',
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        gazebo,
        spawn_entity,
        controller_manager,
        balance_controller,
        sensor_processor
    ])
```

## Performance Optimization

### Physics Parameters
```yaml
# config/physics_params.yaml
gazebo_physics:
  # Time stepping
  max_step_size: 0.001
  real_time_update_rate: 1000
  real_time_factor: 1.0

  # Solver parameters
  solver_type: "ode"
  ode_solver:
    type: "quick"
    iters: 100
    sor: 1.3

  # Constraints
  constraints:
    cfm: 0.0
    erp: 0.2
    contact_max_correcting_vel: 100.0
    contact_surface_layer: 0.001

  # Contacts
  contact:
    collide_without_contact: false
    collide_without_contact_bitmask: 1
    bitmask: 1

  # Gravity
  gravity: [0.0, 0.0, -9.8]
```

## Troubleshooting Guide

### Common Issues and Solutions

1. **Instability in Simulation**
   - Check joint limits and stiffness parameters
   - Verify mass and inertia properties
   - Adjust physics solver parameters
   - Increase simulation frequency

2. **Balance Control Problems**
   - Verify IMU placement and calibration
   - Check ZMP calculation algorithm
   - Tune balance control gains
   - Validate contact detection

3. **Performance Issues**
   - Reduce model complexity
   - Adjust update rates
   - Optimize collision meshes
   - Use simpler physics approximations

4. **Sensor Noise**
   - Calibrate sensor noise parameters
   - Implement filtering algorithms
   - Check sensor placement
   - Verify coordinate frames

## Evaluation Criteria

### Technical Requirements (60%)
- **Model Accuracy**: URDF/XACRO model correctly represents humanoid kinematics
- **Physics Simulation**: Accurate physics behavior matching real-world expectations
- **Control Integration**: Proper integration with ROS 2 control systems
- **Sensor Simulation**: Accurate sensor data generation
- **Stability**: Robot maintains balance under various conditions

### Implementation Quality (25%)
- **Code Quality**: Clean, well-documented, following ROS 2 conventions
- **Modularity**: Well-structured, reusable components
- **Error Handling**: Robust error detection and recovery
- **Performance**: Efficient resource usage and real-time capability

### Testing and Validation (15%)
- **Test Coverage**: Comprehensive testing of all features
- **Validation**: Comparison with expected behavior
- **Documentation**: Clear documentation of setup and usage
- **Reproducibility**: Clear instructions for reproducing results

## Advanced Features

### Optional Enhancements
- **Dynamic Environment**: Moving obstacles and changing terrain
- **Learning-Based Control**: Reinforcement learning for gait optimization
- **Multi-Robot Simulation**: Coordination between multiple humanoid robots
- **Advanced Sensors**: Force/torque sensors, tactile sensors
- **Real-Time Visualization**: Advanced visualization of planning and control

This project provides comprehensive experience in Gazebo simulation for humanoid robotics, covering model creation, physics simulation, sensor integration, and control system development. Students will gain practical experience in creating realistic simulation environments that closely match real-world robot behavior.