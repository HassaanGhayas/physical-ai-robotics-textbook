---
title: URDF for Humanoids
sidebar_position: 4
description: Understanding URDF for describing humanoid robot geometry and kinematics
---

# URDF (Unified Robot Description Format) for Humanoids

## Overview

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF becomes particularly complex due to the articulated nature and multiple degrees of freedom.

## URDF Components for Humanoids

### Links
Represent rigid bodies of the robot:

```xml
<link name="torso">
  <visual>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <capsule length="0.4" radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
  </inertial>
</link>
```

### Joints
Define how links connect and move relative to each other:

```xml
<joint name="torso_head_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <axis xyz="0 1 0"/>
  <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
  <origin xyz="0 0 0.4" rpy="0 0 0"/>
</joint>
```

## Humanoid-Specific Considerations

### 1. Kinematic Chains
Humanoids have complex kinematic chains:
- Left arm: torso → shoulder → elbow → wrist → hand
- Right arm: torso → shoulder → elbow → wrist → hand
- Left leg: torso → hip → knee → ankle → foot
- Right leg: torso → hip → knee → ankle → foot

### 2. Balance and Stability
- Center of Mass (CoM) calculation is critical
- Support polygon considerations for bipedal locomotion
- Inertial properties must be accurate for stable control

### 3. Degrees of Freedom
Humanoids typically have 30+ degrees of freedom:
- 6 DOF per leg (hip: 3DOF, knee: 1DOF, ankle: 2DOF)
- 6 DOF per arm (shoulder: 3DOF, elbow: 1DOF, wrist: 2DOF)
- 3 DOF for torso (waist)
- 3 DOF for head (neck)

## URDF Best Practices for Humanoids

### 1. Hierarchical Structure
Organize URDF files in a hierarchy:
- Base file includes all limbs
- Separate files for each limb
- Common parts in shared files

### 2. Materials and Colors
Use consistent materials for visual clarity:
```xml
<material name="skin">
  <color rgba="0.9 0.8 0.7 1.0"/>
</material>
<material name="metal">
  <color rgba="0.5 0.5 0.5 1.0"/>
</material>
```

### 3. Collision Meshes
Use simplified collision meshes for performance:
- Convex hulls instead of detailed meshes
- Capsules and cylinders instead of complex geometries
- Separate visual and collision models

## Example: Simplified Humanoid URDF

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Torso -->
  <link name="base_link">
    <visual>
      <geometry>
        <capsule length="0.4" radius="0.1"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.4" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.3" iyz="0" izz="0.2"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.008" ixy="0" ixz="0" iyy="0.008" iyz="0" izz="0.008"/>
    </inertial>
  </link>

  <joint name="torso_head_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="100" velocity="1.0"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Similar definitions for arms and legs -->
</robot>
```

## Working with URDF in ROS

### Loading URDF
```python
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def load_urdf():
    urdf_file = os.path.join(
        get_package_share_directory('humanoid_description'),
        'urdf',
        'humanoid.urdf.xacro'
    )
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    return doc.toprettyxml(indent='  ')
```

### URDF Validation
Always validate your URDF:
- Use `check_urdf` command
- Visualize in RViz
- Test with Gazebo simulation

## Troubleshooting Common Issues

### 1. Kinematic Loops
Humanoids with closed chains (e.g., holding an object with both hands) require special handling with `transmission` elements.

### 2. Inertial Properties
Incorrect inertial properties can cause simulation instability. Use CAD tools to calculate accurate values.

### 3. Joint Limits
Proper joint limits prevent unrealistic poses and potential damage to physical robots.