---
title: "VLA Pipeline Architecture"
sidebar_label: "VLA Pipeline"
sidebar_position: 6
description: "Building complete Voice→Language→Action systems with ROS 2"
tags: [vla-pipeline, system-architecture, integration, real-time]
keywords: [VLA pipeline, system architecture, ROS 2 integration, real-time robotics, message passing]
difficulty: advanced
estimated_time: "2.5 hours"
prerequisites: ["Chapter 5: Vision-Language Models"]
---

# VLA Pipeline Architecture

*Content coming soon. This chapter will cover:*

## Complete VLA System Overview

### End-to-End Pipeline

```
🎤 Audio Input (Microphone)
    ↓
🔊 Speech-to-Text (Whisper)
    ↓ "Bring me the red box"
🧠 Language Understanding (LLM)
    ↓ [plan: navigate, detect, grasp, return]
👁️ Visual Grounding (Vision-Language Model)
    ↓ {object_id: "box_0042", pose: [x, y, z]}
📋 Task Planning (Task Planner)
    ↓ [navigate(kitchen), grasp(box_0042), navigate(user)]
🗺️ Motion Planning (Nav2 + MoveIt)
    ↓ {path: [...], trajectory: [...]}
🤖 Execution (Robot Controllers)
    ↓
✅ Feedback & Monitoring
    ↓ (loop back for replanning)
```

## ROS 2 System Architecture

### Node Diagram

```
┌─────────────────┐
│  Audio Capture  │
│      Node       │
└────────┬────────┘
         │ /audio_stream
         ↓
┌─────────────────┐
│  Whisper ASR    │
│      Node       │
└────────┬────────┘
         │ /voice_command (String)
         ↓
┌─────────────────┐
│  LLM Planner    │
│      Node       │
└────────┬────────┘
         │ /task_plan (TaskPlan)
         ↓
┌─────────────────────────────────────┐
│         VLA Coordinator Node        │
│  - Manages execution flow           │
│  - Calls vision-language grounding  │
│  - Interfaces with motion planners  │
└────────┬────────────────────────────┘
         │
    ┌────┴─────┬─────────────┬──────────┐
    │          │             │          │
    ↓          ↓             ↓          ↓
┌────────┐ ┌────────┐ ┌──────────┐ ┌────────┐
│ Vision │ │  Nav2  │ │  MoveIt  │ │  TTS   │
│  Node  │ │        │ │          │ │  Node  │
└────────┘ └────────┘ └──────────┘ └────────┘
```

### Message Types

```python
# Custom ROS 2 messages

# TaskPlan.msg
string command_text
TaskAction[] actions
float32 confidence

# TaskAction.msg
string action_type  # navigate, detect, grasp, place, wait
string[] parameters
geometry_msgs/Pose target_pose
bool requires_vision

# ExecutionStatus.msg
string current_action
float32 progress
string status  # idle, executing, success, failed, replanning
```

## State Management

### System States

```python
class SystemState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING_COMMAND = "processing_command"
    PLANNING = "planning"
    VISUAL_GROUNDING = "visual_grounding"
    EXECUTING = "executing"
    MONITORING = "monitoring"
    RECOVERING = "recovering"
    COMPLETE = "complete"
    ERROR = "error"
```

### State Machine

```python
class VLAStateMachine:
    def __init__(self):
        self.state = SystemState.IDLE
        self.current_plan = None
        self.execution_index = 0

    def transition(self, new_state):
        self.get_logger().info(f"State: {self.state} → {new_state}")
        self.state = new_state

    def process(self):
        if self.state == SystemState.IDLE:
            # Wait for voice command
            if self.voice_command_received:
                self.transition(SystemState.PROCESSING_COMMAND)

        elif self.state == SystemState.PROCESSING_COMMAND:
            # Send to LLM
            self.current_plan = self.llm_planner.plan(self.command)
            self.transition(SystemState.VISUAL_GROUNDING)

        elif self.state == SystemState.VISUAL_GROUNDING:
            # Ground language references to percepts
            self.ground_objects()
            self.transition(SystemState.EXECUTING)

        elif self.state == SystemState.EXECUTING:
            # Execute actions sequentially
            action = self.current_plan[self.execution_index]
            success = self.execute_action(action)
            if success:
                self.execution_index += 1
                if self.execution_index >= len(self.current_plan):
                    self.transition(SystemState.COMPLETE)
            else:
                self.transition(SystemState.RECOVERING)

        # ... more states
```

## VLA Coordinator Node

### Implementation

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vla_interfaces.msg import TaskPlan, TaskAction, ExecutionStatus
import asyncio

class VLACoordinatorNode(Node):
    def __init__(self):
        super().__init__('vla_coordinator')

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, 'voice_command', self.voice_callback, 10
        )

        # Publishers
        self.status_pub = self.create_publisher(ExecutionStatus, 'execution_status', 10)
        self.speech_pub = self.create_publisher(String, 'robot_speech', 10)

        # Service clients
        self.llm_client = self.create_client(PlanTask, 'llm_plan')
        self.vision_client = self.create_client(GroundObject, 'ground_object')
        self.nav_client = self.create_client(NavigateToPose, 'navigate_to_pose')

        # State
        self.state_machine = VLAStateMachine()

        # Timer for processing state machine
        self.timer = self.create_timer(0.1, self.process_state_machine)

    def voice_callback(self, msg):
        self.get_logger().info(f"Received command: {msg.data}")
        self.state_machine.voice_command_received = True
        self.state_machine.command = msg.data

    async def execute_task_plan(self, plan):
        for action in plan.actions:
            success = await self.execute_action(action)
            if not success:
                # Replan or recover
                self.robot_say("I encountered a problem. Replanning...")
                new_plan = await self.replan(plan, action)
                await self.execute_task_plan(new_plan)
                return

        self.robot_say("Task complete!")

    async def execute_action(self, action):
        if action.action_type == "navigate":
            return await self.navigate(action.target_pose)
        elif action.action_type == "grasp":
            return await self.grasp_object(action.parameters[0])
        elif action.action_type == "detect":
            return await self.detect_object(action.parameters[0])
        # ... more action types

    def robot_say(self, text):
        msg = String()
        msg.data = text
        self.speech_pub.publish(msg)
```

## Real-Time Constraints

### Latency Budget

| Component | Target Latency | Notes |
|-----------|---------------|-------|
| Voice capture | 100ms | Continuous streaming |
| Whisper ASR | 500-1000ms | Depends on model size |
| LLM planning | 1-3s | Can be async |
| Vision grounding | 200-500ms | GPU-accelerated |
| Motion planning | 500-2000ms | Varies by complexity |
| **Total** | **3-7s** | Command to first action |

### Optimization Strategies

**1. Parallel Processing**

```python
# Don't wait for full plan before starting vision
async def optimized_pipeline():
    # Start vision processing early
    vision_task = asyncio.create_task(capture_and_process_image())

    # While vision runs, get LLM plan
    plan = await get_llm_plan(command)

    # Vision results should be ready by now
    grounded_objects = await vision_task

    # Execute with both ready
    execute_plan(plan, grounded_objects)
```

**2. Predictive Processing**

```python
# Start processing likely next actions
if current_action == "navigate":
    # Likely next: detect or grasp
    preload_vision_model()
    start_object_detection()
```

**3. Caching**

```python
# Cache LLM results for repeated commands
command_cache = {}

def get_plan(command):
    if command in command_cache:
        return command_cache[command]
    plan = llm.plan(command)
    command_cache[command] = plan
    return plan
```

## Error Handling & Recovery

### Error Types

**1. Perception Failures**
```python
if not detected_objects:
    robot_say("I can't find that object. Could you point it out?")
    # Wait for user gesture or clarification
```

**2. Execution Failures**
```python
if not grasp_successful:
    # Retry with different approach
    alternative_grasp = plan_alternative_grasp(object)
    retry_grasp(alternative_grasp)
```

**3. Safety Violations**
```python
if action.violates_safety_constraint():
    emergency_stop()
    robot_say("I can't do that, it's not safe")
    return_to_safe_state()
```

### Recovery Behaviors

```python
class RecoveryManager:
    def recover(self, failure):
        if failure.type == "object_not_found":
            return self.search_wider_area()
        elif failure.type == "path_blocked":
            return self.replan_path()
        elif failure.type == "grasp_failed":
            return self.retry_grasp_different_angle()
        elif failure.type == "timeout":
            return self.ask_for_help()
        else:
            return self.abort_and_reset()
```

## Logging and Debugging

### Structured Logging

```python
class VLALogger:
    def log_command(self, command):
        log_entry = {
            "timestamp": time.time(),
            "type": "command",
            "text": command,
            "user_id": self.current_user
        }
        self.write_log(log_entry)

    def log_action(self, action, status, duration):
        log_entry = {
            "timestamp": time.time(),
            "type": "action",
            "action": action.to_dict(),
            "status": status,
            "duration_ms": duration * 1000
        }
        self.write_log(log_entry)

    def log_failure(self, action, error):
        log_entry = {
            "timestamp": time.time(),
            "type": "failure",
            "action": action.to_dict(),
            "error": str(error),
            "stack_trace": traceback.format_exc()
        }
        self.write_log(log_entry)
```

### Visualization in RViz2

```python
# Publish debug markers for visualization
def visualize_plan(plan):
    marker_array = MarkerArray()

    for i, action in enumerate(plan):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.id = i
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = f"{i}: {action.action_type}"
        marker.pose = action.target_pose
        marker_array.markers.append(marker)

    self.marker_pub.publish(marker_array)
```

## Performance Monitoring

### Metrics to Track

```python
class PerformanceMonitor:
    def __init__(self):
        self.metrics = {
            "commands_received": 0,
            "commands_completed": 0,
            "commands_failed": 0,
            "avg_completion_time": 0,
            "llm_calls": 0,
            "llm_total_time": 0,
            "vision_calls": 0,
            "vision_total_time": 0
        }

    def record_command_completion(self, duration, success):
        self.metrics["commands_received"] += 1
        if success:
            self.metrics["commands_completed"] += 1
        else:
            self.metrics["commands_failed"] += 1

        # Update rolling average
        n = self.metrics["commands_completed"]
        self.metrics["avg_completion_time"] = (
            (self.metrics["avg_completion_time"] * (n-1) + duration) / n
        )

    def get_success_rate(self):
        total = self.metrics["commands_received"]
        if total == 0:
            return 0.0
        return self.metrics["commands_completed"] / total
```

### Prometheus Integration

```python
from prometheus_client import Counter, Histogram, start_http_server

# Define metrics
commands_total = Counter('vla_commands_total', 'Total commands received')
command_duration = Histogram('vla_command_duration_seconds', 'Command execution time')
llm_latency = Histogram('vla_llm_latency_seconds', 'LLM response time')

# Use in code
commands_total.inc()
with command_duration.time():
    execute_command(cmd)
```

## Multi-Modal Synchronization

### Temporal Alignment

```python
class MultiModalBuffer:
    def __init__(self, sync_threshold=0.1):
        self.sync_threshold = sync_threshold
        self.buffers = {
            "audio": [],
            "video": [],
            "robot_state": []
        }

    def add_sample(self, modality, timestamp, data):
        self.buffers[modality].append((timestamp, data))
        self.cleanup_old_data()

    def get_synchronized_data(self, target_time):
        """Get data from all modalities at approximately target_time"""
        synced = {}
        for modality, buffer in self.buffers.items():
            closest = min(buffer, key=lambda x: abs(x[0] - target_time))
            if abs(closest[0] - target_time) < self.sync_threshold:
                synced[modality] = closest[1]
        return synced if len(synced) == len(self.buffers) else None
```

## Testing and Validation

### Unit Tests

```python
import pytest

def test_command_parsing():
    command = "Bring me the red box"
    parsed = parse_command(command)
    assert parsed["action"] == "bring"
    assert parsed["object"]["color"] == "red"
    assert parsed["object"]["type"] == "box"

def test_safety_validation():
    dangerous_command = "Throw the battery in water"
    assert not validate_safety(dangerous_command)

    safe_command = "Place the box on the table"
    assert validate_safety(safe_command)
```

### Integration Tests

```python
@pytest.mark.integration
async def test_full_vla_pipeline():
    # Mock components
    mock_llm = MockLLMPlanner()
    mock_vision = MockVisionSystem()
    mock_robot = MockRobotExecutor()

    coordinator = VLACoordinator(mock_llm, mock_vision, mock_robot)

    # Execute command
    result = await coordinator.execute_command("Move forward 1 meter")

    # Verify
    assert result.success
    assert mock_robot.final_position.x == pytest.approx(1.0, abs=0.1)
```

### Simulation Testing

```python
# Test in Isaac Sim before real robot
def test_in_simulation():
    sim = IsaacSimulator()
    robot = sim.spawn_robot()

    vla_system = VLASystem(robot)

    commands = [
        "Navigate to the kitchen",
        "Find the red cup",
        "Bring it to me"
    ]

    for cmd in commands:
        success = vla_system.execute(cmd)
        assert success, f"Failed on command: {cmd}"
```

## Hands-On Projects

### Project 1: Build the Coordinator
- Implement VLA coordinator node
- Connect Whisper, LLM, vision, and execution
- Test with simple commands

### Project 2: State Machine
- Implement full state machine
- Handle transitions and errors
- Add recovery behaviors

### Project 3: Performance Optimization
- Profile the pipeline
- Identify bottlenecks
- Implement parallel processing

### Project 4: Complete System Test
- Deploy on real/simulated robot
- Test with 20+ different commands
- Measure success rate and latency

*For now, study ROS 2 best practices and explore [ROS 2 composition](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html).*
