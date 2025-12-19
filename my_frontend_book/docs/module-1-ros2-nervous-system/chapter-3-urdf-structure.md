---
title: Robot Structure with URDF
description: Learn about URDF (Unified Robot Description Format) for humanoid robots and simulation readiness
sidebar_label: Robot Structure with URDF
sidebar_position: 4
tags: [urdf, robot-description, humanoid-robotics, simulation]
authors: [Spec-Kit Plus Team]
keywords: [URDF, robot description, humanoid, simulation, links, joints]
---

# Robot Structure with URDF

## URDF Fundamentals

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It's the standard way to describe robot structure in ROS and ROS 2, defining the physical and visual properties of a robot.

### XML Structure and Syntax

URDF files are XML documents with a specific structure:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="base_to_top" type="fixed">
    <parent link="base_link"/>
    <child link="top_link"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <link name="top_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```

### Links (Rigid Bodies)

Links represent rigid bodies in the robot:

- **Visual Elements**: How the link appears in simulation and visualization
- **Collision Elements**: How the link interacts with the environment in physics simulation
- **Inertial Elements**: Physical properties like mass and moment of inertia
- **Unique Names**: Each link must have a unique name within the robot

### Joints (Connections)

Joints connect links and define their relative motion:

- **Joint Types**: Fixed, continuous, revolute, prismatic, floating, planar
- **Parent-Child Relationships**: Each joint connects exactly two links
- **Joint Limits**: For revolute and prismatic joints, define range of motion
- **Joint Origins**: Position and orientation of the joint relative to parent

### Visual and Collision Elements

URDF distinguishes between visual and collision properties:

- **Visual**: How the robot appears in simulation and visualization
- **Collision**: How the robot interacts with the environment in physics simulation
- **Geometry Types**: Box, cylinder, sphere, mesh, or custom shapes
- **Materials**: Color and appearance properties

## Humanoid-Specific Considerations

Humanoid robots have unique structural characteristics that require special attention in URDF.

### Multiple Degrees of Freedom

Humanoid robots typically have many joints:

- **Legs**: Hip, knee, ankle joints for locomotion
- **Arms**: Shoulder, elbow, wrist joints for manipulation
- **Spine**: Multiple joints for posture and balance
- **Head**: Neck joints for vision and interaction

### Symmetrical Structures

Humanoid robots are typically symmetrical:

```xml
<!-- Right arm -->
<link name="right_shoulder">
  <!-- ... -->
</link>

<!-- Left arm (mirror of right) -->
<link name="left_shoulder">
  <!-- ... -->
</link>
```

### Kinematic Chains

Humanoid robots form complex kinematic chains:

- **Leg Chains**: Hip → Knee → Ankle → Foot
- **Arm Chains**: Shoulder → Elbow → Wrist → Hand
- **Spine Chain**: Multiple vertebrae links
- **Closed Chains**: When multiple chains connect to the same point

### End Effectors

Hands are crucial for humanoid manipulation:

- **Finger Joints**: Multiple small joints for dexterity
- **Grasp Points**: Special markers for grasp planning
- **Sensors**: Integration of tactile sensors in fingertips
- **Actuators**: Motors or pneumatic systems for finger movement

### Sensor Integration

Humanoid robots have many sensors:

- **IMUs**: Inertial measurement units for balance
- **Cameras**: For vision-based perception
- **Force/Torque Sensors**: In feet and hands
- **Joint Sensors**: Encoders for position feedback

## Simulation Readiness

For URDF to be simulation-ready, several requirements must be met.

### Complete Kinematic Chain

All links and joints must be properly connected:

- **Root Link**: One link must serve as the root of the kinematic tree
- **Connected Structure**: All links must be reachable from the root
- **No Cycles**: The kinematic structure must be a tree (no loops)
- **Proper Joint Definitions**: All joints must have parent and child links

### Valid Physics Properties

Physics simulation requires accurate physical properties:

- **Mass Values**: Each link must have a realistic mass value
- **Inertial Tensors**: Proper moment of inertia values
- **Friction Coefficients**: Realistic friction for contact simulation
- **Damping Values**: Appropriate damping for smooth simulation

### Collision Models

Proper collision detection requires:

- **Collision Meshes**: Accurate meshes for physics simulation
- **Simplified Geometry**: Sometimes simplified shapes for performance
- **Contact Surfaces**: Properly defined surfaces for interaction
- **No Intersections**: Collision meshes shouldn't intersect when at rest

### Visual Models

For rendering and visualization:

- **Visual Meshes**: High-quality meshes for appearance
- **Textures and Materials**: Proper appearance properties
- **Transparency**: Support for transparent parts where needed
- **LOD**: Level of detail for performance optimization

### Transmission Elements

To connect actuators to joints:

```xml
<transmission name="left_elbow_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_elbow_joint">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_elbow_motor">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Practical URDF Examples

Here's a simple humanoid leg example:

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg">
  <!-- Torso (root link) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Hip joint and upper leg -->
  <joint name="right_hip_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.1 0 -0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <link name="right_upper_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Knee joint and lower leg -->
  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="0" upper="2.35" effort="100" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <!-- Ankle joint and foot -->
  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_lower_leg"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="1"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
</robot>
```

## Validation and Best Practices

### URDF Validation

Always validate your URDF files:

- **XML Validation**: Check for proper XML syntax
- **URDF Validation**: Use tools like `check_urdf` to validate structure
- **Simulation Testing**: Test in simulation before real robot deployment
- **Visualization**: Check that the robot appears correctly in RViz

### Best Practices

- **Use Xacro**: For complex robots, use Xacro to reduce repetition
- **Modular Design**: Break complex robots into modular parts
- **Realistic Values**: Use realistic physical properties
- **Documentation**: Comment your URDF files for clarity
- **Testing**: Test URDF with various simulation scenarios

## Summary

URDF is the standard format for describing robot structure in ROS/ROS 2. For humanoid robots, special attention must be paid to multiple degrees of freedom, symmetrical structures, and complex kinematic chains. Making URDF simulation-ready requires proper kinematic chains, valid physics properties, and appropriate collision and visual models. Following best practices ensures your humanoid robot descriptions are accurate, efficient, and ready for both simulation and real-world deployment.