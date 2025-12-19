---
title: URDF Specification Reference
description: Comprehensive reference for URDF (Unified Robot Description Format) specification
sidebar_label: URDF Specification Reference
sidebar_position: 8
tags: [urdf, reference, robot-description, xml]
authors: [Spec-Kit Plus Team]
keywords: [URDF, robot description, specification, xml, reference]
---

# URDF Specification Reference

This reference provides comprehensive information about the URDF (Unified Robot Description Format) specification. Use this as a quick lookup for URDF elements, attributes, and best practices.

## Overview

URDF (Unified Robot Description Format) is an XML format for representing a robot model. It describes the physical and visual properties of a robot, including links, joints, materials, and other elements.

### Basic Structure

A URDF file follows this basic structure:

```xml
<?xml version="1.0"?>
<robot name="robot_name">
  <!-- Links defining rigid bodies -->
  <link name="link_name">
    <!-- Visual properties -->
    <visual>
      <!-- Geometry definition -->
      <geometry>
        <!-- Shape definition -->
      </geometry>
      <!-- Material definition -->
    </visual>
    <!-- Collision properties -->
    <collision>
      <geometry>
        <!-- Shape definition -->
      </geometry>
    </collision>
    <!-- Inertial properties -->
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Joints defining connections between links -->
  <joint name="joint_name" type="joint_type">
    <parent link="parent_link_name"/>
    <child link="child_link_name"/>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <!-- Joint limits for revolute joints -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>

  <!-- Materials for visualization -->
  <material name="material_name">
    <color rgba="r g b a"/>
  </material>
</robot>
```

## Links

A link represents a rigid body in the robot model. Each link can have visual, collision, and inertial properties.

### Link Attributes
- **name**: Unique identifier for the link within the robot (required)

### Link Elements

#### Visual Element
Defines how the link appears in visualization and rendering.

**Attributes:**
- **optional**: `name` - Optional name for the visual element

**Sub-elements:**
- **geometry**: Defines the shape of the visual element
- **material**: Defines the appearance (color, texture)
- **origin**: Position and orientation relative to the link frame

#### Collision Element
Defines how the link interacts with the environment in physics simulation.

**Attributes:**
- **optional**: `name` - Optional name for the collision element

**Sub-elements:**
- **geometry**: Defines the shape of the collision element
- **origin**: Position and orientation relative to the link frame

#### Inertial Element
Defines the physical properties for dynamics simulation.

**Sub-elements:**
- **mass**: Mass of the link (required)
- **inertia**: Inertia matrix (required)

### Geometry Types

#### Box
A rectangular parallelepiped.

```xml
<geometry>
  <box size="length width height"/>
</geometry>
```
- **size**: Three values (x, y, z) representing the dimensions

#### Cylinder
A cylinder with the length along the Z-axis.

```xml
<geometry>
  <cylinder radius="0.1" length="0.2"/>
</geometry>
```
- **radius**: Radius of the cylinder
- **length**: Length of the cylinder

#### Sphere
A sphere.

```xml
<geometry>
  <sphere radius="0.1"/>
</geometry>
```
- **radius**: Radius of the sphere

#### Mesh
A custom mesh defined in an external file.

```xml
<geometry>
  <mesh filename="package://package_name/meshes/mesh_file.dae" scale="1 1 1"/>
</geometry>
```
- **filename**: Path to the mesh file (supports package:// URLs)
- **scale**: Optional scale factors (x, y, z)

## Joints

A joint connects two links and defines their relative motion.

### Joint Attributes
- **name**: Unique identifier for the joint (required)
- **type**: Type of joint (required)

### Joint Types

#### Fixed
No relative motion between the links.

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

#### Revolute
Rotational motion around a single axis.

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

#### Continuous
Continuous rotational motion (like a revolute joint without limits).

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

#### Prismatic
Linear motion along a single axis.

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.5" effort="100" velocity="1"/>
</joint>
```

#### Floating
Six degrees of freedom with no constraints.

```xml
<joint name="floating_joint" type="floating">
  <parent link="parent_link"/>
  <child link="child_link"/>
</joint>
```

#### Planar
Motion in a plane.

```xml
<joint name="planar_joint" type="planar">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <axis xyz="0 0 1"/>
</joint>
```

### Joint Elements

#### Parent
Specifies the parent link in the kinematic tree.

```xml
<parent link="parent_link_name"/>
```

#### Child
Specifies the child link in the kinematic tree.

```xml
<child link="child_link_name"/>
```

#### Origin
Specifies the position and orientation of the joint relative to the parent link.

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```
- **xyz**: Position as x, y, z coordinates
- **rpy**: Orientation as roll, pitch, yaw angles in radians

#### Axis
Specifies the axis of motion for revolute and prismatic joints.

```xml
<axis xyz="x y z"/>
```
- **xyz**: Unit vector defining the axis of rotation or translation

#### Limit
Defines the limits for revolute and prismatic joints.

```xml
<limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
```
- **lower**: Lower limit in radians or meters
- **upper**: Upper limit in radians or meters
- **effort**: Maximum effort (torque or force) in N or Nm
- **velocity**: Maximum velocity in rad/s or m/s

#### Safety Controller
Defines safety limits for the joint.

```xml
<safety_controller k_position="100" k_velocity="10" soft_lower_limit="-1.0" soft_upper_limit="1.0"/>
```
- **k_position**: Position gain
- **k_velocity**: Velocity gain
- **soft_lower_limit**: Soft lower limit
- **soft_upper_limit**: Soft upper limit

## Materials

Materials define the appearance of links in visualization.

### Material Attributes
- **name**: Unique identifier for the material (required)

### Material Elements

#### Color
Defines the RGBA color of the material.

```xml
<color rgba="red green blue alpha"/>
```
- **rgba**: Four values (r, g, b, a) from 0 to 1

#### Texture
Defines a texture image for the material.

```xml
<texture filename="package://package_name/textures/texture.png"/>
```
- **filename**: Path to the texture file (supports package:// URLs)

## Transmissions

Transmissions define how actuators connect to joints.

### Transmission Types

#### Simple Transmission
Connects one actuator to one joint.

```xml
<transmission name="simple_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint_name">
    <hardwareInterface>PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="actuator_name">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

#### Differential Transmission
Connects two actuators to two joints (e.g., for differential drive).

```xml
<transmission name="differential_trans">
  <type>transmission_interface/DifferentialTransmission</type>
  <joint name="left_joint">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
  </joint>
  <joint name="right_joint">
    <hardwareInterface>VelocityJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_actuator">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
  <actuator name="right_actuator">
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## Gazebo Elements

Gazebo-specific elements can be added to URDF files.

### Gazebo Reference
Applies Gazebo-specific properties to a link.

```xml
<gazebo reference="link_name">
  <material>Gazebo/Blue</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

### Gazebo Plugins
Add plugins to simulate sensors or controllers.

```xml
<gazebo>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
  </plugin>
</gazebo>
```

## Xacro (XML Macros)

Xacro is an XML macro language that extends URDF with macros and expressions.

### Including Xacro Files

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="robot_name">
  <xacro:include filename="$(find package_name)/urdf/common.xacro"/>
</robot>
```

### Properties

```xml
<xacro:property name="wheel_radius" value="0.1"/>
<xacro:property name="M_PI" value="3.1415926535897931"/>
```

### Macros

```xml
<xacro:macro name="wheel" params="prefix">
  <link name="${prefix}_wheel">
    <visual>
      <geometry>
        <cylinder radius="${wheel_radius}" length="0.05"/>
      </geometry>
    </visual>
  </link>
</xacro:macro>

<!-- Use the macro -->
<xacro:wheel prefix="front"/>
<xacro:wheel prefix="rear"/>
```

## Common URDF Elements and Attributes

### Robot Element
- **name**: Name of the robot (required)

### Link Element
- **name**: Name of the link (required)

### Joint Element
- **name**: Name of the joint (required)
- **type**: Type of joint (required)

### Mass Element
- **value**: Mass value in kilograms (required)

### Inertia Element
- **ixx, ixy, ixz, iyy, iyz, izz**: Inertia matrix values (required)

### Origin Element
- **xyz**: Position vector (x, y, z)
- **rpy**: Orientation as roll, pitch, yaw in radians

## Inertial Properties

For proper physics simulation, each link needs accurate inertial properties.

### Mass
The mass of the link in kilograms.

### Inertia Matrix
A 3x3 symmetric matrix representing the distribution of mass:

```
| ixx  ixy  ixz |
| ixy  iyy  iyz |
| ixz  iyz  izz |
```

### Common Inertia Formulas

#### Box (mass m, dimensions x, y, z)
- ixx = m * (y² + z²) / 12
- iyy = m * (x² + z²) / 12
- izz = m * (x² + y²) / 12

#### Cylinder (mass m, radius r, length l) - about central axis
- ixx = m * (3*r² + l²) / 12
- iyy = m * (3*r² + l²) / 12
- izz = m * r² / 2

#### Sphere (mass m, radius r)
- ixx = iyy = izz = 2 * m * r² / 5

## Best Practices

### Naming Conventions
- Use descriptive, consistent names
- Use underscores to separate words
- Follow a consistent pattern (e.g., left_wheel, right_wheel)

### Structure
- Start with a single root link
- Ensure all links are connected in a tree structure
- Avoid kinematic loops (use mechanisms for closed chains)

### Validation
- Ensure all links have proper inertial properties
- Verify joint limits are realistic
- Check that all referenced links exist
- Validate URDF with `check_urdf` tool

### Performance
- Use simple collision geometries when possible
- Use meshes only for visualization, not collision
- Keep the number of links reasonable for performance

### Organization
- Use Xacro to reduce repetition
- Break complex robots into modular parts
- Use include files for common components

## Common Issues and Troubleshooting

### Kinematic Structure
- Ensure all links are reachable from the root
- Verify there are no disconnected parts
- Check for proper parent-child relationships

### Inertial Properties
- Verify all links have mass values
- Check that inertia values are physically plausible
- Use positive values for mass and diagonal inertia terms

### Joint Configuration
- Ensure joint limits are appropriate for the physical robot
- Verify axis directions are correct
- Check that joint types match intended motion

### Visualization
- Verify all links have visual elements for display
- Check that materials are properly defined
- Ensure mesh files are accessible

## Validation Tools

### Command Line Validation
```bash
# Check URDF syntax and structure
check_urdf /path/to/robot.urdf

# Check Xacro files
xacro /path/to/robot.xacro > /tmp/robot.urdf && check_urdf /tmp/robot.urdf
```

### Visualization
- Use RViz to visualize the robot model
- Use Gazebo to test physics simulation
- Use `rqt_robot_monitor` to check robot state

## Example: Complete Robot

```xml
<?xml version="1.0"?>
<robot name="example_robot">
  <!-- Root link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.02"/>
    </inertial>
  </link>

  <!-- Wheel joint -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Wheel link -->
  <link name="wheel_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>
</robot>
```

## Summary

This reference provides comprehensive information about the URDF specification and its elements. Use this as a guide when creating robot models for simulation and visualization. Remember to validate your URDF files and test them in simulation before using them with real robots.