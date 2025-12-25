---
sidebar_position: 4
title: Chapter 3 - Humanoid Robot Description with URDF
---

# Chapter 3: Humanoid Robot Description with URDF

## The Purpose of URDF in Humanoid Robotics

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF serves as the blueprint that defines the robot's physical structure, including links (rigid parts), joints (connections between parts), and other properties like visual appearance and collision geometry.

URDF is crucial because it bridges the gap between software algorithms and physical robot hardware. Without an accurate robot description, algorithms can't understand the robot's structure, limiting the effectiveness of control, planning, and simulation.

## Links, Joints, and Coordinate Frames

### Links
Links represent the rigid parts of a robot. In a humanoid robot, examples include:
- Torso
- Upper arm
- Lower arm
- Hand
- Thigh
- Shin
- Foot

Each link has properties like:
- Mass
- Inertia
- Visual representation
- Collision representation

### Joints
Joints define how links connect and move relative to each other. Common joint types include:
- **Revolute**: Rotational joint with a single axis of rotation
- **Prismatic**: Linear sliding joint
- **Fixed**: No movement between links
- **Continuous**: Rotational joint without limits
- **Planar**: Movement on a plane
- **Floating**: 6 degrees of freedom

### Coordinate Frames
Each link has its own coordinate frame, typically defined at the joint connecting it to its parent. These frames establish the spatial relationship between different parts of the robot and are essential for:
- Kinematics calculations
- Motion planning
- Sensor data interpretation

## How URDF Connects Software to Physical Structure

URDF files serve multiple purposes in the software-to-hardware pipeline:

1. **Simulation**: URDF models are used in physics simulators like Gazebo
2. **Visualization**: Tools like RViz use URDF to visualize the robot
3. **Kinematics**: Forward and inverse kinematics algorithms use URDF structure
4. **Collision Detection**: URDF collision meshes are used for safety checks
5. **Control**: Joint limits and properties from URDF inform control algorithms

## High-Level Explanation of Humanoid Kinematics

Humanoid kinematics involves understanding how joint angles relate to end-effector positions (like hand or foot positions). There are two main problems:

### Forward Kinematics
Given joint angles, calculate the position and orientation of end-effectors. This is relatively straightforward mathematically.

### Inverse Kinematics
Given a desired end-effector position, calculate the required joint angles. This is more complex and may have multiple solutions or no solution.

For humanoid robots, kinematics is particularly important for:
- Walking and balance control
- Manipulation tasks
- Motion planning
- Trajectory generation

## A Basic URDF Example

Here's a simplified URDF for a 2-joint robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- First joint -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Second joint -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="end_effector"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- End effector -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## Preparing Robot Models for Simulation

To prepare URDF models for simulation in Gazebo or other simulators, you typically need to add additional elements:

### Transmission Elements
These define how actuators connect to joints:

```xml
<transmission name="joint1_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="joint1_motor">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Gazebo-Specific Elements
These define how the model appears and behaves in Gazebo:

```xml
<gazebo reference="base_link">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Isaac Sim Readiness

For use with Isaac Sim (NVIDIA's robotics simulator), URDF files may need additional processing or conversion to USD (Universal Scene Description) format. Isaac Sim also supports some URDF directly but may require:

- Proper joint limits and types
- Collision and visual mesh definitions
- Material specifications
- Inertial properties for physics simulation

<details>
<summary>Advanced: Xacro for Complex Robots</summary>

For complex robots with many repeated elements, Xacro (XML Macros) is used to simplify URDF creation:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="complex_robot">
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <xacro:macro name="simple_arm" params="prefix parent *origin">
    <joint name="${prefix}_joint" type="revolute">
      <xacro:insert_block name="origin"/>
      <parent link="${parent}"/>
      <child link="${prefix}_link"/>
      <axis xyz="0 1 0"/>
      <limit lower="${-2*M_PI}" upper="${2*M_PI}" effort="30" velocity="1.0"/>
    </joint>

    <link name="${prefix}_link">
      <visual>
        <geometry>
          <cylinder radius="0.02" length="0.5"/>
        </geometry>
        <origin xyz="0 0 0.25" rpy="0 0 0"/>
      </visual>
    </link>
  </xacro:macro>
</robot>
```
</details>

## Summary

In this chapter, you've learned about URDF (Unified Robot Description Format) and its crucial role in connecting software algorithms to physical robot structure. You now understand how links, joints, and coordinate frames define a robot's structure, how URDF enables various robotics applications, and the basics of humanoid kinematics. You've also seen how to prepare robot models for simulation in Gazebo and Isaac Sim.

With all three chapters complete, you now have a comprehensive understanding of ROS 2 as the robotic nervous system, how to connect Python AI agents to ROS 2, and how robots are described using URDF.