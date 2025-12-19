---
sidebar_position: 3
title: "Chapter 2: Digital Twins & HRI in Unity"
---

# Chapter 2: Digital Twins & HRI in Unity

This chapter focuses on creating high-fidelity digital twins using Unity and implementing Human-Robot Interaction (HRI) interfaces. Unity provides powerful visualization capabilities that complement physics simulation from Gazebo.

## Learning Objectives

By the end of this chapter, you will be able to:
- Set up Unity for robotics simulation
- Import and configure 3D robot models in Unity
- Design effective Human-Robot Interaction interfaces
- Implement animation systems for humanoid robots

## Introduction to Unity for Robotics

Unity is a powerful game engine that has found significant applications in robotics for creating high-fidelity digital twins and visualization. Unity's advantages for robotics include:

- High-quality real-time rendering
- Flexible UI system for HRI
- Physics simulation capabilities
- Extensive asset store with robotics resources
- Cross-platform deployment options

## Setting Up Unity for Robotics

Unity can be enhanced for robotics applications using the Unity Robotics Hub, which includes:

- Unity Robotics Simulation (URS) for physics simulation
- ROS# for ROS/ROS 2 communication
- ML-Agents for reinforcement learning
- Various robotics packages and samples

### Installing Unity

1. Download Unity Hub from the Unity website
2. Install Unity Editor (2022.3 LTS recommended)
3. Install required packages via Unity Package Manager

### Unity Robotics Setup

To set up Unity for robotics simulation:

1. Create a new 3D project
2. Install ROS# package for ROS communication
3. Configure network settings for ROS bridge
4. Set up appropriate physics settings

## Importing 3D Robot Models

Unity supports various 3D model formats (FBX, OBJ, DAE). To import a robot model:

```csharp
// Example: Loading a robot model programmatically
using UnityEngine;

public class RobotLoader : MonoBehaviour
{
    public GameObject robotModel;

    void Start()
    {
        // Load robot model from resources
        robotModel = Resources.Load<GameObject>("RobotModel");
        Instantiate(robotModel, Vector3.zero, Quaternion.identity);
    }
}
```

### Converting from URDF to Unity

While Unity doesn't directly support URDF, you can:
1. Export URDF as COLLADA (DAE) format
2. Import DAE files into Unity
3. Manually configure joints and constraints in Unity
4. Use the Unity Robotics package for ROS integration

## Human-Robot Interaction (HRI) Design

Effective HRI design is crucial for human-robot collaboration. Unity provides excellent tools for creating intuitive interfaces:

### UI System in Unity

Unity's UI system allows for creating interactive interfaces:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class HRIInterface : MonoBehaviour
{
    public Button startButton;
    public Slider speedSlider;
    public Text statusText;

    void Start()
    {
        startButton.onClick.AddListener(OnStartClicked);
        speedSlider.onValueChanged.AddListener(OnSpeedChanged);
    }

    void OnStartClicked()
    {
        statusText.text = "Robot started";
        // Send command to robot via ROS
    }

    void OnSpeedChanged(float value)
    {
        statusText.text = "Speed: " + value.ToString("F2");
        // Update robot speed via ROS
    }
}
```

### Interaction Patterns

Common HRI patterns in Unity include:

1. **Direct Manipulation**: Users can directly interact with robot models
2. **Command Interface**: Buttons and sliders for controlling robot behavior
3. **Visualization**: Real-time feedback on robot state and sensor data
4. **Augmented Reality**: Overlaying information on robot views

## Animation Systems for Humanoid Robots

Unity's Animation system is powerful for creating realistic humanoid robot movements:

### Setting up the Animator

```csharp
using UnityEngine;

public class RobotAnimator : MonoBehaviour
{
    private Animator animator;
    public float walkSpeed;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // Control animations based on robot state
        animator.SetFloat("Speed", walkSpeed);
        animator.SetBool("IsWalking", walkSpeed > 0.1f);
    }
}
```

### Mecanim for Humanoid Animation

Unity's Mecanim system provides advanced animation features:
- Animation layers for blending different movements
- State machines for complex animation logic
- Humanoid rig for retargeting animations
- IK (Inverse Kinematics) for realistic movement

## Practical Exercise: Creating a Basic Robot Interface

Create a simple Unity scene with a robot model and basic HRI interface:

1. Import a simple robot model into Unity
2. Create a UI canvas with control buttons
3. Implement basic robot control logic
4. Set up ROS communication (using ROS#)

### Basic Robot Control Script

```csharp
using UnityEngine;
using System.Collections;

// This is a simplified example
// In practice, you'd use ROS# for actual ROS communication
public class RobotController : MonoBehaviour
{
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 100.0f;

    void Update()
    {
        // Simple movement controls
        float translation = Input.GetAxis("Vertical") * moveSpeed * Time.deltaTime;
        float rotation = Input.GetAxis("Horizontal") * rotateSpeed * Time.deltaTime;

        transform.Translate(0, 0, translation);
        transform.Rotate(0, rotation, 0);
    }
}
```

## Integration with ROS

Unity can communicate with ROS/ROS 2 systems using the ROS# package:

1. Install ROS# in your Unity project
2. Configure IP addresses for ROS bridge
3. Create publishers and subscribers
4. Send/receive messages to control the robot

## Best Practices

- Optimize 3D models for real-time rendering
- Use appropriate physics settings for stable simulation
- Design intuitive UIs for HRI applications
- Implement proper error handling for ROS communication
- Consider performance implications of complex visualizations

## Summary

In this chapter, you learned how to create high-fidelity digital twins in Unity and implement effective HRI interfaces. You explored Unity's capabilities for robotics visualization and learned to create intuitive interfaces for human-robot interaction. In the next chapter, we'll focus on sensor simulation and validation techniques.