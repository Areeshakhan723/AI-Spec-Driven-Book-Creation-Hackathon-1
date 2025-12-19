---
sidebar_position: 6
title: "Unity Digital Twin Tutorial"
---

# Unity Digital Twin Tutorial

This tutorial will guide you through creating a digital twin of a humanoid robot in Unity and implementing basic Human-Robot Interaction (HRI) interfaces.

## Prerequisites

Before starting this tutorial, you should:
- Have Unity Hub and Unity Editor (2022.3 LTS or newer) installed
- Basic understanding of C# programming
- Basic knowledge of 3D modeling concepts

## Setting up Unity for Robotics

### Step 1: Install Unity Robotics Hub

1. Open Unity Hub
2. Go to the "Packages" tab
3. Install "Unity Robotics Hub" from the package list
4. This includes:
   - Unity Robotics Simulation (URS)
   - ROS# communication package
   - Sample scenes and tutorials

### Step 2: Create a New Unity Project

1. Open Unity Hub
2. Click "New Project"
3. Select the "3D (Built-in Render Pipeline)" template
4. Name your project "HumanoidRobotDigitalTwin"
5. Click "Create"

### Step 3: Import Robotics Packages

1. In Unity, go to Window → Package Manager
2. Install "ROS Communication" package
3. Install "Unity Robotics Simulation" package

## Creating a Basic Robot Model

### Step 1: Create Robot Components

Let's create a simple humanoid robot using basic Unity primitives:

```csharp
using UnityEngine;

public class SimpleHumanoidRobot : MonoBehaviour
{
    [Header("Body Parts")]
    public GameObject head;
    public GameObject torso;
    public GameObject leftArm;
    public GameObject rightArm;
    public GameObject leftLeg;
    public GameObject rightLeg;

    [Header("Joint Positions")]
    public Transform neckJoint;
    public Transform shoulderLeft;
    public Transform shoulderRight;
    public Transform hipLeft;
    public Transform hipRight;

    void Start()
    {
        CreateRobotStructure();
    }

    void CreateRobotStructure()
    {
        // Create torso (main body)
        torso = GameObject.CreatePrimitive(PrimitiveType.Cube);
        torso.name = "Torso";
        torso.transform.localScale = new Vector3(0.5f, 0.8f, 0.3f);
        torso.transform.SetParent(transform);
        torso.GetComponent<Renderer>().material.color = Color.blue;

        // Create head
        head = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        head.name = "Head";
        head.transform.localScale = new Vector3(0.25f, 0.25f, 0.25f);
        head.transform.position = new Vector3(0, 0.65f, 0) + torso.transform.position;
        head.GetComponent<Renderer>().material.color = Color.white;

        // Create arms
        leftArm = CreateLimb("LeftArm", new Vector3(-0.4f, 0.2f, 0) + torso.transform.position,
                            new Vector3(0.1f, 0.4f, 0.1f), Color.gray);
        rightArm = CreateLimb("RightArm", new Vector3(0.4f, 0.2f, 0) + torso.transform.position,
                             new Vector3(0.1f, 0.4f, 0.1f), Color.gray);

        // Create legs
        leftLeg = CreateLimb("LeftLeg", new Vector3(-0.15f, -0.6f, 0) + torso.transform.position,
                            new Vector3(0.15f, 0.5f, 0.15f), Color.red);
        rightLeg = CreateLimb("RightLeg", new Vector3(0.15f, -0.6f, 0) + torso.transform.position,
                             new Vector3(0.15f, 0.5f, 0.15f), Color.red);
    }

    GameObject CreateLimb(string name, Vector3 position, Vector3 scale, Color color)
    {
        GameObject limb = GameObject.CreatePrimitive(PrimitiveType.Capsule);
        limb.name = name;
        limb.transform.position = position;
        limb.transform.localScale = scale;
        limb.GetComponent<Renderer>().material.color = color;
        return limb;
    }
}
```

### Step 2: Create the Robot Controller

Create a script called `RobotController.cs` to control the robot:

```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 2.0f;
    public float turnSpeed = 50.0f;

    [Header("Animation Settings")]
    public float walkCycleSpeed = 2.0f;
    public float armSwingAmount = 0.5f;
    public float legSwingAmount = 0.3f;

    private Animator animator;
    private GameObject head;
    private GameObject leftArm, rightArm;
    private GameObject leftLeg, rightLeg;

    private float walkCycle = 0.0f;
    private bool isWalking = false;

    void Start()
    {
        FindRobotParts();
        SetupAnimator();
    }

    void FindRobotParts()
    {
        // Find robot parts in the hierarchy
        Transform torso = transform.Find("Torso");
        if (torso != null)
        {
            head = torso.Find("Head")?.gameObject;
            leftArm = torso.Find("LeftArm")?.gameObject;
            rightArm = torso.Find("RightArm")?.gameObject;
            leftLeg = transform.Find("LeftLeg")?.gameObject;
            rightLeg = transform.Find("RightLeg")?.gameObject;
        }
    }

    void SetupAnimator()
    {
        // If using Mecanim, set up the animator
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        HandleInput();
        UpdateAnimation();
    }

    void HandleInput()
    {
        float translation = Input.GetAxis("Vertical") * moveSpeed * Time.deltaTime;
        float rotation = Input.GetAxis("Horizontal") * turnSpeed * Time.deltaTime;

        // Move the robot
        transform.Translate(0, 0, translation);
        transform.Rotate(0, rotation, 0);

        // Update walking state
        isWalking = Mathf.Abs(translation) > 0.1f || Mathf.Abs(rotation) > 0.1f;
    }

    void UpdateAnimation()
    {
        if (isWalking)
        {
            // Update walk cycle
            walkCycle += walkCycleSpeed * Time.deltaTime;

            // Simple walking animation by swinging arms and legs
            if (leftArm != null && rightArm != null && leftLeg != null && rightLeg != null)
            {
                // Swing arms opposite to leg movement
                leftArm.transform.localRotation = Quaternion.Euler(
                    Mathf.Sin(walkCycle) * armSwingAmount * 30f,
                    0,
                    Mathf.Cos(walkCycle) * armSwingAmount * 10f
                );

                rightArm.transform.localRotation = Quaternion.Euler(
                    -Mathf.Sin(walkCycle) * armSwingAmount * 30f,
                    0,
                    -Mathf.Cos(walkCycle) * armSwingAmount * 10f
                );

                // Swing legs
                leftLeg.transform.localRotation = Quaternion.Euler(
                    Mathf.Sin(walkCycle * 2) * legSwingAmount * 20f,
                    0,
                    0
                );

                rightLeg.transform.localRotation = Quaternion.Euler(
                    -Mathf.Sin(walkCycle * 2) * legSwingAmount * 20f,
                    0,
                    0
                );
            }
        }
        else
        {
            // Reset limb rotations when not walking
            if (leftArm != null) leftArm.transform.localRotation = Quaternion.identity;
            if (rightArm != null) rightArm.transform.localRotation = Quaternion.identity;
            if (leftLeg != null) leftLeg.transform.localRotation = Quaternion.identity;
            if (rightLeg != null) rightLeg.transform.localRotation = Quaternion.identity;
        }
    }
}
```

## Creating HRI (Human-Robot Interaction) Interface

### Step 1: Set up the UI Canvas

1. In Unity, go to GameObject → UI → Canvas
2. This creates a Canvas, Panel, and Text element

### Step 2: Create HRI Control Script

Create a script called `HRIController.cs`:

```csharp
using UnityEngine;
using UnityEngine.UI;

public class HRIController : MonoBehaviour
{
    [Header("UI Elements")]
    public Button moveForwardButton;
    public Button moveBackwardButton;
    public Button turnLeftButton;
    public Button turnRightButton;
    public Slider speedSlider;
    public Text statusText;
    public Text speedText;

    [Header("Robot Reference")]
    public RobotController robotController;

    void Start()
    {
        SetupUI();
    }

    void SetupUI()
    {
        if (moveForwardButton) moveForwardButton.onClick.AddListener(MoveForward);
        if (moveBackwardButton) moveBackwardButton.onClick.AddListener(MoveBackward);
        if (turnLeftButton) turnLeftButton.onClick.AddListener(TurnLeft);
        if (turnRightButton) turnRightButton.onClick.AddListener(TurnRight);

        if (speedSlider) speedSlider.onValueChanged.AddListener(OnSpeedChanged);

        UpdateSpeedText();
    }

    void MoveForward()
    {
        if (robotController)
        {
            // Send command to robot to move forward
            statusText.text = "Moving Forward";
            // In a real implementation, this would send a command via ROS
        }
    }

    void MoveBackward()
    {
        if (robotController)
        {
            statusText.text = "Moving Backward";
        }
    }

    void TurnLeft()
    {
        if (robotController)
        {
            statusText.text = "Turning Left";
        }
    }

    void TurnRight()
    {
        if (robotController)
        {
            statusText.text = "Turning Right";
        }
    }

    void OnSpeedChanged(float value)
    {
        if (robotController)
        {
            robotController.moveSpeed = value;
        }
        UpdateSpeedText();
    }

    void UpdateSpeedText()
    {
        if (speedSlider && speedText)
        {
            speedText.text = $"Speed: {speedSlider.value:F1}";
        }
    }
}
```

### Step 3: Connect UI to Robot

1. Create an empty GameObject called "RobotManager"
2. Add the `SimpleHumanoidRobot` script to it
3. Add the `RobotController` script to it
4. Create another empty GameObject called "HRIInterface"
5. Add the `HRIController` script to it
6. In the HRIController component, drag the UI elements and RobotManager to their respective fields

## Adding Camera Views

### Step 1: Create Multiple Camera Views

```csharp
using UnityEngine;

public class CameraManager : MonoBehaviour
{
    [Header("Camera Types")]
    public Camera mainCamera;
    public Camera robotCamera;  // First-person view from robot
    public Camera overviewCamera;  // Top-down view

    [Header("Camera Switching")]
    public KeyCode mainCamKey = KeyCode.Alpha1;
    public KeyCode robotCamKey = KeyCode.Alpha2;
    public KeyCode overviewCamKey = KeyCode.Alpha3;

    void Start()
    {
        SetupCameras();
    }

    void SetupCameras()
    {
        // Get references to cameras if not assigned in inspector
        if (mainCamera == null) mainCamera = Camera.main;

        // Disable additional cameras initially
        if (robotCamera != null) robotCamera.enabled = false;
        if (overviewCamera != null) overviewCamera.enabled = false;

        // Enable main camera
        if (mainCamera != null) mainCamera.enabled = true;
    }

    void Update()
    {
        if (Input.GetKeyDown(mainCamKey))
        {
            SwitchToMainCamera();
        }
        else if (Input.GetKeyDown(robotCamKey))
        {
            SwitchToRobotCamera();
        }
        else if (Input.GetKeyDown(overviewCamKey))
        {
            SwitchToOverviewCamera();
        }
    }

    void SwitchToMainCamera()
    {
        DisableAllCameras();
        if (mainCamera != null) mainCamera.enabled = true;
    }

    void SwitchToRobotCamera()
    {
        DisableAllCameras();
        if (robotCamera != null)
        {
            robotCamera.enabled = true;
            // Position robot camera at head position
            robotCamera.transform.position = GetRobotHeadPosition();
        }
    }

    void SwitchToOverviewCamera()
    {
        DisableAllCameras();
        if (overviewCamera != null)
        {
            overviewCamera.enabled = true;
            // Position overview camera above the scene
            overviewCamera.transform.position = new Vector3(0, 10, 0);
            overviewCamera.transform.rotation = Quaternion.Euler(90, 0, 0);
        }
    }

    void DisableAllCameras()
    {
        if (mainCamera != null) mainCamera.enabled = false;
        if (robotCamera != null) robotCamera.enabled = false;
        if (overviewCamera != null) overviewCamera.enabled = false;
    }

    Vector3 GetRobotHeadPosition()
    {
        // In a real implementation, this would get the head position
        return transform.position + new Vector3(0, 1.5f, 0);
    }
}
```

## Integrating with ROS (Optional)

If you want to connect your Unity simulation to ROS, you can use ROS#:

### Step 1: Install ROS# Package

1. In Unity, go to Window → Package Manager
2. Install the "ROS Communication" package

### Step 2: Create ROS Communication Script

```csharp
using System.Collections;
using UnityEngine;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.RosSharp;

public class ROSRobotController : MonoBehaviour
{
    private RosConnector rosConnector;
    private string robotName = "unity_robot";

    void Start()
    {
        InitializeROS();
    }

    void InitializeROS()
    {
        // Initialize ROS connection
        rosConnector = GetComponent<RosConnector>();

        if (rosConnector != null)
        {
            rosConnector.RosSocketUrl = "ws://192.168.1.100:9090"; // Update with your ROS bridge IP
            rosConnector.ConnectToRos();
        }
    }

    public void SendVelocityCommand(float linearX, float angularZ)
    {
        if (rosConnector != null && rosConnector.IsConnected)
        {
            // Create and send Twist message
            TwistMsg twist = new TwistMsg();
            twist.linear = new Vector3Msg(linearX, 0, 0);
            twist.angular = new Vector3Msg(0, 0, angularZ);

            rosConnector.RosSocket.Publish($"/{robotName}/cmd_vel", twist);
        }
    }

    void OnDestroy()
    {
        if (rosConnector != null)
        {
            rosConnector.DisconnectFromRos();
        }
    }
}
```

## Building and Testing

### Step 1: Test in Editor

1. Press Play in Unity Editor
2. Use arrow keys to control the robot
3. Test the HRI interface buttons
4. Switch between camera views

### Step 2: Build for Deployment

1. Go to File → Build Settings
2. Select your target platform (Windows, Linux, etc.)
3. Add your scene to the build
4. Click "Build" to create an executable

## Summary

In this tutorial, you learned how to:
- Set up Unity for robotics applications
- Create a simple humanoid robot model
- Implement basic movement and animation
- Create HRI interfaces with Unity's UI system
- Set up multiple camera views
- Optionally integrate with ROS

This provides a foundation for creating more complex digital twins and HRI interfaces in Unity.