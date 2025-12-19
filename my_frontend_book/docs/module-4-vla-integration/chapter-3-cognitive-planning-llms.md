---
sidebar_position: 4
title: "Chapter 3: Cognitive Planning (LLMs)"
---

# Chapter 3: Cognitive Planning (LLMs)

This chapter focuses on using Large Language Models (LLMs) for cognitive planning in robotics. You'll learn how to translate natural language commands into detailed action plans that can be executed by ROS 2 systems, enabling robots to understand and execute complex, multi-step tasks.

## Learning Objectives

By the end of this chapter, you will be able to:
- Integrate LLMs with ROS 2 for cognitive planning
- Translate natural language commands to structured action plans
- Implement task decomposition using LLMs
- Execute action plans with Nav2 and other ROS 2 components

## Introduction to Cognitive Planning

Cognitive planning in robotics refers to the process of converting high-level goals or commands into executable sequences of actions. LLMs excel at this task because they can:

- Understand natural language commands
- Reason about the world and robot capabilities
- Break down complex tasks into manageable steps
- Adapt plans based on environmental context

### The Cognitive Planning Pipeline

The cognitive planning pipeline consists of:

1. **Command Understanding**: Interpreting the natural language command
2. **Context Integration**: Incorporating environmental and robot state information
3. **Plan Generation**: Creating a sequence of executable actions
4. **Plan Validation**: Ensuring the plan is safe and feasible
5. **Plan Execution**: Executing the plan through ROS 2 action servers

## LLM Integration with ROS 2

### Setting up LLM Clients

First, let's establish the LLM client interface for robotics applications:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import ActionPlan, ActionStep
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
import json
import time

class LLMClient:
    def __init__(self, model_type="openai", api_key=None, model_name="gpt-3.5-turbo"):
        self.model_type = model_type
        self.model_name = model_name

        if model_type == "openai":
            import openai
            openai.api_key = api_key
            self.client = openai.OpenAI(api_key=api_key)
        elif model_type == "anthropic":
            import anthropic
            self.client = anthropic.Anthropic(api_key=api_key)
        elif model_type == "local":
            # Setup for local models like Llama
            from transformers import pipeline
            self.client = pipeline("text-generation", model="meta-llama/Llama-2-7b-chat-hf")

    def generate_plan(self, prompt, max_tokens=500):
        """Generate a plan from a prompt using the LLM"""
        if self.model_type == "openai":
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=max_tokens,
                temperature=0.3
            )
            return response.choices[0].message.content
        elif self.model_type == "anthropic":
            response = self.client.messages.create(
                model=self.model_name,
                max_tokens=max_tokens,
                temperature=0.3,
                messages=[{"role": "user", "content": prompt}]
            )
            return response.content[0].text
        else:
            # Local model implementation
            result = self.client(prompt, max_length=max_tokens, temperature=0.3)[0]['generated_text']
            return result[len(prompt):]  # Return only the generated part
```

### Cognitive Planning Node

```python
class CognitivePlanningNode(Node):
    def __init__(self):
        super().__init__('cognitive_planning_node')

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            'natural_language_command',
            self.command_callback,
            10
        )

        self.environment_sub = self.create_subscription(
            String,
            'environment_context',
            self.environment_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(
            ActionPlan,
            'generated_action_plan',
            10
        )

        # LLM client initialization
        self.llm_client = LLMClient(
            model_type="openai",  # Change as needed
            api_key="your-api-key-here",
            model_name="gpt-3.5-turbo"
        )

        # Store environment context
        self.environment_context = ""
        self.robot_capabilities = self.get_robot_capabilities()

        self.get_logger().info("Cognitive planning node initialized")

    def get_robot_capabilities(self):
        """Define robot capabilities for LLM context"""
        return {
            "navigation": {
                "enabled": True,
                "max_speed": 0.5,  # m/s
                "min_turn_radius": 0.3  # m
            },
            "manipulation": {
                "enabled": True,
                "max_reach": 1.0,  # m
                "gripper_type": "parallel_jaw"
            },
            "perception": {
                "enabled": True,
                "camera_resolution": "640x480",
                "object_detection": True
            }
        }

    def command_callback(self, msg):
        """Process natural language command and generate action plan"""
        command = msg.data
        self.get_logger().info(f"Processing command: {command}")

        # Create prompt with context
        prompt = self.create_planning_prompt(command, self.environment_context)

        try:
            # Generate plan using LLM
            llm_response = self.llm_client.generate_plan(prompt)

            # Parse LLM response into structured action plan
            action_plan = self.parse_llm_response(llm_response, command)

            # Publish the action plan
            self.plan_pub.publish(action_plan)

            self.get_logger().info(f"Generated action plan with {len(action_plan.steps)} steps")

        except Exception as e:
            self.get_logger().error(f"Error generating plan: {e}")

    def create_planning_prompt(self, command, environment_context):
        """Create a structured prompt for cognitive planning"""
        prompt = f"""
        You are a robot cognitive planner. Your task is to convert the user's natural language command
        into a detailed sequence of executable robot actions.

        Robot Capabilities:
        - Navigation: The robot can move to specific locations
        - Manipulation: The robot can pick up, place, and manipulate objects
        - Perception: The robot can detect and identify objects using cameras
        - Communication: The robot can provide status updates

        Robot Specifications:
        {json.dumps(self.robot_capabilities, indent=2)}

        Current Environment:
        {environment_context}

        Natural Language Command: {command}

        Please provide a detailed action plan in JSON format with the following structure:
        {{
            "command": "original command",
            "steps": [
                {{
                    "step_number": 1,
                    "action_type": "navigation|manipulation|perception|other",
                    "description": "detailed description of the action",
                    "parameters": {{"key": "value"}},
                    "preconditions": ["list of preconditions that must be true"],
                    "expected_outcome": "description of expected result"
                }}
            ],
            "estimated_duration": "estimated time in seconds",
            "confidence": "confidence level 0-1"
        }}

        Each action should be specific, executable, and safe. Consider environmental constraints
        and robot capabilities. If the command is ambiguous, ask for clarification rather than
        making assumptions.
        """
        return prompt

    def parse_llm_response(self, response, original_command):
        """Parse LLM response into structured ActionPlan message"""
        try:
            # Extract JSON from response (in case LLM adds extra text)
            start_idx = response.find('{')
            end_idx = response.rfind('}') + 1

            if start_idx != -1 and end_idx != 0:
                json_str = response[start_idx:end_idx]
                plan_data = json.loads(json_str)
            else:
                # If no JSON found, try to parse the whole response
                plan_data = json.loads(response)

            # Create ActionPlan message
            action_plan = ActionPlan()
            action_plan.command = original_command
            action_plan.estimated_duration = plan_data.get('estimated_duration', 0.0)
            action_plan.confidence = plan_data.get('confidence', 0.5)
            action_plan.timestamp = self.get_clock().now().to_msg()

            # Convert steps to ActionStep messages
            for step_data in plan_data.get('steps', []):
                step_msg = ActionStep()
                step_msg.step_number = step_data.get('step_number', 0)
                step_msg.action_type = step_data.get('action_type', 'other')
                step_msg.description = step_data.get('description', '')
                step_msg.parameters = json.dumps(step_data.get('parameters', {}))
                step_msg.preconditions = step_data.get('preconditions', [])
                step_msg.expected_outcome = step_data.get('expected_outcome', '')

                action_plan.steps.append(step_msg)

            return action_plan

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing LLM response as JSON: {e}")
            # Create a simple fallback plan
            return self.create_fallback_plan(original_command, response)
        except Exception as e:
            self.get_logger().error(f"Error parsing LLM response: {e}")
            return self.create_fallback_plan(original_command, response)

    def create_fallback_plan(self, original_command, response):
        """Create a fallback plan when JSON parsing fails"""
        action_plan = ActionPlan()
        action_plan.command = original_command
        action_plan.estimated_duration = 0.0
        action_plan.confidence = 0.1  # Low confidence for fallback
        action_plan.timestamp = self.get_clock().now().to_msg()

        # Create a single step with the raw response
        step_msg = ActionStep()
        step_msg.step_number = 1
        step_msg.action_type = 'other'
        step_msg.description = f"Fallback: {response[:100]}..."  # Truncate if too long
        step_msg.parameters = json.dumps({})
        step_msg.preconditions = []
        step_msg.expected_outcome = "Unknown - parsing failed"

        action_plan.steps.append(step_msg)
        return action_plan

    def environment_callback(self, msg):
        """Update environment context"""
        self.environment_context = msg.data
```

## Task Decomposition with LLMs

### Multi-Step Task Planning

LLMs excel at breaking down complex tasks into manageable steps:

```python
class TaskDecompositionNode(Node):
    def __init__(self):
        super().__init__('task_decomposition_node')

        # Subscribers
        self.complex_task_sub = self.create_subscription(
            String,
            'complex_task_command',
            self.complex_task_callback,
            10
        )

        # Publishers
        self.subtask_pub = self.create_publisher(
            ActionPlan,
            'decomposed_task_plan',
            10
        )

        # LLM client
        self.llm_client = LLMClient(
            model_type="openai",
            api_key="your-api-key-here"
        )

    def complex_task_callback(self, msg):
        """Decompose complex tasks into subtasks"""
        task = msg.data

        # Create decomposition prompt
        prompt = self.create_decomposition_prompt(task)

        try:
            response = self.llm_client.generate_plan(prompt)
            subtask_plan = self.parse_decomposition_response(response, task)
            self.subtask_pub.publish(subtask_plan)

        except Exception as e:
            self.get_logger().error(f"Error decomposing task: {e}")

    def create_decomposition_prompt(self, task):
        """Create prompt for task decomposition"""
        prompt = f"""
        Decompose the following complex task into smaller, executable subtasks:

        Task: {task}

        Consider the following:
        - Each subtask should be achievable by a robot with basic capabilities
        - Subtasks should follow logical sequence
        - Include necessary preconditions and verification steps
        - Consider safety and feasibility

        Provide the decomposition in JSON format:
        {{
            "original_task": "{task}",
            "subtasks": [
                {{
                    "id": 1,
                    "name": "subtask name",
                    "description": "detailed description",
                    "dependencies": [list of dependent subtask IDs],
                    "estimated_time": "time in seconds",
                    "required_capabilities": ["list", "of", "required", "capabilities"]
                }}
            ],
            "execution_order": [sequence of subtask IDs to execute]
        }}
        """
        return prompt
```

## Natural Language to Action Translation

### Semantic Understanding

LLMs can understand the semantics of natural language commands and map them to appropriate actions:

```python
class SemanticCommandMapper:
    def __init__(self):
        self.action_templates = {
            'navigation': [
                'go to {location}',
                'move to {location}',
                'navigate to {location}',
                'head to {location}',
                'travel to {location}'
            ],
            'manipulation': [
                'pick up {object}',
                'grasp {object}',
                'take {object}',
                'get {object}',
                'lift {object}',
                'place {object} at {location}',
                'put {object} on {location}'
            ],
            'perception': [
                'find {object}',
                'locate {object}',
                'search for {object}',
                'detect {object}',
                'look for {object}',
                'identify {object}'
            ],
            'action': [
                'clean {location}',
                'organize {location}',
                'arrange {location}',
                'tidy {location}'
            ]
        }

    def map_command_to_action(self, command, llm_client):
        """Use LLM to map natural language to specific actions"""
        prompt = f"""
        Map the following natural language command to specific robotic actions:

        Command: {command}

        Available action types:
        - navigation: Moving to locations
        - manipulation: Picking up, placing, or handling objects
        - perception: Detecting or identifying objects
        - action: Complex multi-step operations

        Provide the mapping in JSON format:
        {{
            "command": "{command}",
            "action_type": "navigation|manipulation|perception|action",
            "target_object": "object to act on (if applicable)",
            "target_location": "location to act on (if applicable)",
            "parameters": {{"additional": "parameters"}},
            "reasoning": "explain why this mapping makes sense"
        }}
        """

        response = llm_client.generate_plan(prompt)
        return self.parse_mapping_response(response)
```

## Integration with Nav2 and ROS 2 Execution

### Plan Execution Node

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class PlanExecutionNode(Node):
    def __init__(self):
        super().__init__('plan_execution_node')

        # Subscribers
        self.plan_sub = self.create_subscription(
            ActionPlan,
            'generated_action_plan',
            self.plan_callback,
            10
        )

        # Publishers
        self.status_pub = self.create_publisher(
            String,
            'execution_status',
            10
        )

        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Plan queue
        self.current_plan = None
        self.current_step_index = 0
        self.execution_active = False

        self.get_logger().info("Plan execution node initialized")

    def plan_callback(self, msg):
        """Receive and execute action plan"""
        self.get_logger().info(f"Received plan with {len(msg.steps)} steps")

        if self.execution_active:
            self.get_logger().warn("Plan execution already active, queuing new plan")
            # Implement queueing if needed
            return

        self.current_plan = msg
        self.current_step_index = 0
        self.execution_active = True

        # Start execution
        self.execute_next_step()

    def execute_next_step(self):
        """Execute the next step in the plan"""
        if not self.current_plan or self.current_step_index >= len(self.current_plan.steps):
            self.get_logger().info("Plan execution completed")
            self.execution_active = False
            self.publish_status("Plan completed")
            return

        current_step = self.current_plan.steps[self.current_step_index]
        self.get_logger().info(f"Executing step {current_step.step_number}: {current_step.action_type}")

        # Execute based on action type
        if current_step.action_type == 'navigation':
            self.execute_navigation_step(current_step)
        elif current_step.action_type == 'manipulation':
            self.execute_manipulation_step(current_step)
        elif current_step.action_type == 'perception':
            self.execute_perception_step(current_step)
        else:
            self.execute_other_step(current_step)

    def execute_navigation_step(self, step):
        """Execute navigation step"""
        try:
            # Parse parameters for navigation
            params = json.loads(step.parameters)
            target_location = params.get('target_location', '')

            # Convert location to pose (this would use a location map)
            pose = self.location_to_pose(target_location)

            if pose:
                self.navigate_to_pose(pose)
            else:
                self.get_logger().error(f"Unknown location: {target_location}")
                self.mark_step_completed()

        except Exception as e:
            self.get_logger().error(f"Error executing navigation step: {e}")
            self.mark_step_completed()

    def navigate_to_pose(self, pose):
        """Navigate to a specific pose using Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.nav_client.wait_for_server()
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
        """Handle navigation result"""
        goal_result = future.result()

        if goal_result.status == 4:  # SUCCESS
            self.get_logger().info("Navigation completed successfully")
            self.mark_step_completed()
        else:
            self.get_logger().error(f"Navigation failed with status: {goal_result.status}")
            # Handle failure - maybe skip to next step or abort plan
            self.mark_step_completed()

    def location_to_pose(self, location):
        """Convert location name to PoseStamped"""
        # This would typically use a map of known locations
        location_map = {
            'kitchen': (1.0, 2.0, 0.0),
            'living room': (0.0, 0.0, 0.0),
            'bedroom': (-1.0, 1.0, 1.57),
            'office': (2.0, -1.0, -1.57),
            'dining room': (1.5, -1.5, 0.785)
        }

        if location in location_map:
            x, y, theta = location_map[location]
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            # Convert theta to quaternion
            import math
            cy = math.cos(theta * 0.5)
            sy = math.sin(theta * 0.5)
            pose.pose.orientation.w = cy
            pose.pose.orientation.z = sy
            return pose

        return None

    def mark_step_completed(self):
        """Mark current step as completed and move to next"""
        self.current_step_index += 1
        if self.current_step_index < len(self.current_plan.steps):
            # Schedule next step after a short delay
            self.create_timer(0.5, self.execute_next_step)
        else:
            # All steps completed
            self.get_logger().info("All steps completed")
            self.execution_active = False
            self.publish_status("Plan execution completed")

    def publish_status(self, status):
        """Publish execution status"""
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)
```

## Context-Aware Planning

### Environmental Context Integration

LLMs can incorporate environmental context to make better planning decisions:

```python
class ContextAwarePlanner(Node):
    def __init__(self):
        super().__init__('context_aware_planner')

        # Subscribers for various context sources
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.object_detection_sub = self.create_subscription(
            String,
            '/object_detection_results',
            self.object_detection_callback,
            10
        )

        # Store context information
        self.map_data = None
        self.obstacles = []
        self.detected_objects = {}
        self.last_update_time = self.get_clock().now()

    def get_environment_context(self):
        """Compile environmental context for LLM planning"""
        context = {
            "timestamp": self.get_clock().now().to_msg().sec,
            "map_info": self.get_map_info(),
            "obstacles": self.get_obstacle_info(),
            "objects": self.get_detected_objects(),
            "robot_state": self.get_robot_state(),
            "safety_constraints": self.get_safety_constraints()
        }
        return json.dumps(context, indent=2)

    def get_map_info(self):
        """Extract relevant map information"""
        if self.map_data:
            return {
                "resolution": self.map_data.info.resolution,
                "width": self.map_data.info.width,
                "height": self.map_data.info.height,
                "origin": {
                    "x": self.map_data.info.origin.position.x,
                    "y": self.map_data.info.origin.position.y
                }
            }
        return {}

    def get_obstacle_info(self):
        """Get obstacle information from laser scan"""
        # This would process laser scan data to identify obstacles
        return self.obstacles

    def get_detected_objects(self):
        """Get detected objects from perception system"""
        return self.detected_objects

    def get_robot_state(self):
        """Get current robot state"""
        # This would interface with robot state publisher
        return {
            "position": {"x": 0.0, "y": 0.0, "theta": 0.0},
            "battery_level": 0.8,
            "current_task": "idle"
        }

    def get_safety_constraints(self):
        """Define safety constraints for planning"""
        return {
            "min_distance_to_obstacles": 0.3,  # meters
            "max_navigation_speed": 0.5,  # m/s
            "restricted_areas": [],
            "emergency_stop_conditions": ["collision_imminent", "battery_low"]
        }
```

## Practical Exercise: Complete Cognitive Planning System

Set up a complete cognitive planning system:

1. Configure LLM access and parameters
2. Implement context-aware planning
3. Test with multi-step commands
4. Validate plan execution safety

### Example Command Processing

```python
def example_command_processing():
    """
    Example of processing a complex command:
    "Go to the kitchen, find a red cup, and bring it to the dining table"
    """
    command = "Go to the kitchen, find a red cup, and bring it to the dining table"

    # Expected LLM output would be something like:
    expected_plan = {
        "command": command,
        "steps": [
            {
                "step_number": 1,
                "action_type": "navigation",
                "description": "Navigate to kitchen area",
                "parameters": {"target_location": "kitchen"},
                "preconditions": [],
                "expected_outcome": "Robot is in kitchen area"
            },
            {
                "step_number": 2,
                "action_type": "perception",
                "description": "Search for red cup in kitchen",
                "parameters": {"object_type": "cup", "color": "red"},
                "preconditions": ["robot_in_kitchen"],
                "expected_outcome": "Red cup detected and located"
            },
            {
                "step_number": 3,
                "action_type": "manipulation",
                "description": "Grasp the red cup",
                "parameters": {"object_id": "red_cup_01"},
                "preconditions": ["red_cup_detected", "arm_free"],
                "expected_outcome": "Red cup grasped by robot"
            },
            {
                "step_number": 4,
                "action_type": "navigation",
                "description": "Navigate to dining table",
                "parameters": {"target_location": "dining table"},
                "preconditions": ["cup_grasped"],
                "expected_outcome": "Robot is at dining table with cup"
            },
            {
                "step_number": 5,
                "action_type": "manipulation",
                "description": "Place cup on dining table",
                "parameters": {"target_location": "dining table"},
                "preconditions": ["at_dining_table", "cup_grasped"],
                "expected_outcome": "Cup placed on dining table"
            }
        ],
        "estimated_duration": 120.0,
        "confidence": 0.85
    }

    return expected_plan
```

## Error Handling and Plan Adaptation

### Plan Recovery Strategies

```python
class PlanRecoveryNode(Node):
    def __init__(self):
        super().__init__('plan_recovery_node')

        # Subscribe to execution status and errors
        self.status_sub = self.create_subscription(
            String,
            'execution_status',
            self.status_callback,
            10
        )

        self.error_sub = self.create_subscription(
            String,
            'execution_error',
            self.error_callback,
            10
        )

    def error_callback(self, msg):
        """Handle execution errors and adapt plan"""
        error_msg = msg.data
        self.get_logger().warn(f"Execution error: {error_msg}")

        # Implement recovery strategies
        if "navigation_failed" in error_msg:
            self.handle_navigation_failure()
        elif "object_not_found" in error_msg:
            self.handle_object_detection_failure()
        elif "manipulation_failed" in error_msg:
            self.handle_manipulation_failure()

    def handle_navigation_failure(self):
        """Handle navigation failure by replanning"""
        self.get_logger().info("Attempting navigation recovery")
        # Request new plan from cognitive planner with updated context
        # This might involve obstacle avoidance or alternative routes

    def handle_object_detection_failure(self):
        """Handle object detection failure"""
        self.get_logger().info("Attempting object detection recovery")
        # Request to search in different area or with different parameters

    def handle_manipulation_failure(self):
        """Handle manipulation failure"""
        self.get_logger().info("Attempting manipulation recovery")
        # Request alternative manipulation approach
```

## Summary

In this chapter, you learned how to use LLMs for cognitive planning in robotics, including natural language to action translation, task decomposition, and integration with ROS 2 execution systems. You explored context-aware planning and error recovery strategies. In the next chapter, we'll bring everything together in the capstone project: The Autonomous Humanoid.