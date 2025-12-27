# Chapter 3: Cognitive Planning with LLMs

## Introduction

Imagine telling a robot "clean the room." A human immediately understands this involves locating objects out of place, picking them up, and organizing them appropriately. But how does a robot bridge the gap between this simple instruction and the dozens of motor commands required to execute it? This is the challenge of **cognitive planning**—decomposing high-level natural language goals into executable action sequences.

Traditional robotic planners like PDDL (Planning Domain Definition Language) require explicit domain definitions, action preconditions, and effect specifications. Creating these for real-world environments is labor-intensive and brittle—a new object type or task variation often requires manual updates to the planning domain.

Large Language Models offer a revolutionary alternative. Trained on vast corpora of text describing how humans accomplish tasks, LLMs encode **implicit knowledge** about action sequences, dependencies, and common-sense physics. Ask GPT-4 "what steps are needed to make a sandwich?" and it generates a reasonable procedure without any explicit programming. This capability transfers directly to robotics—LLMs can decompose robot tasks into action sequences that integrate with existing navigation, manipulation, and perception systems.

In this chapter, we'll build a cognitive planning system that translates natural language commands into ROS 2 action sequences. We'll implement an LLM-powered planner with few-shot prompting, an action executor that tracks progress and handles failures, and an error recovery system that re-plans when things go wrong. By the end, you'll have a robot that can interpret "pick up the red cup from the table" and execute the complete sequence: navigate → detect → grasp.

## Foundational Concepts

### Cognitive Planning Fundamentals

**Simple Explanation**: Cognitive planning is the ability to break down a goal ("clean the room") into steps (find objects, pick them up, put them away, repeat). Humans do this automatically; robots need explicit programming or, with LLMs, can learn it from language.

**Detailed Explanation**: Cognitive planning in robotics involves three core capabilities:

1. **Task Decomposition**: Breaking high-level goals into primitive actions the robot can execute
2. **Sequencing**: Ordering actions correctly (can't place an object before grasping it)
3. **Contingency Handling**: Adapting when actions fail or conditions change

Traditional approaches use hierarchical task networks (HTN) or behavior trees with manually specified decompositions. LLMs provide data-driven decomposition—they've "seen" thousands of descriptions of how tasks are accomplished and can generate novel decompositions for new goals.

The planning cycle follows this pattern:

```
Goal → LLM Decomposition → Action Sequence → Execution → Monitor → [Success | Re-plan]
```

**Action Primitives**: The robot's vocabulary of executable actions. Common primitives include:
- **Navigate**: Move to a location (uses navigation stack like Nav2 from [Module 3](../module-03-isaac-navigation/06-nav2-basics.md))
- **Detect**: Find and localize an object (uses computer vision)
- **Grasp**: Pick up an object (uses manipulation stack like MoveIt2)
- **Place**: Put an object down at a location
- **Speak**: Provide verbal feedback
- **Wait**: Pause execution

### LLM Prompt Engineering for Planning

**Simple Explanation**: We tell the LLM "you are a robot planner" and show it examples of commands and their action sequences. Then when we give it a new command, it generates a similar structured plan.

**Detailed Explanation**: Effective LLM planning requires careful prompt engineering:

**1. System Prompt**: Defines the LLM's role and output format
```
You are a robot task planner. Given a high-level command,
break it down into executable actions.

Available actions: navigate, detect, grasp, place, speak, wait

Output JSON format:
{
    "goal": "<command>",
    "plan": [{"step": 1, "type": "<action>", ...}],
    "confidence": 0.0-1.0
}
```

**2. Few-Shot Examples**: Show the LLM correct decompositions
```
Command: "Pick up the cup"
→ [navigate to table, detect cup, grasp cup]

Command: "Clean the room"
→ [scan room, navigate to object, grasp, navigate to bin, place, repeat]
```

**3. Structured Output**: Request JSON for reliable parsing
```json
{
    "plan": [
        {"step": 1, "type": "navigate", "target": "table"},
        {"step": 2, "type": "detect", "target": "cup"},
        {"step": 3, "type": "grasp", "target": "cup"}
    ]
}
```

**4. Temperature Control**: Use low temperature (0.0-0.2) for deterministic, consistent plans

**Best Practices**:
- Include 3-5 diverse examples covering simple and complex tasks
- Specify all valid action types and their parameters
- Request confidence scores for uncertainty handling
- Include estimated durations for progress tracking

### Planning Architectures

**Simple Explanation**: There are different ways to connect the LLM planner to the robot—from simple "plan once, execute all" to sophisticated "plan, execute, observe, re-plan" loops.

**Detailed Explanation**: Planning architectures vary in their feedback integration:

**1. Open-Loop Planning**: Generate complete plan, execute sequentially
```
Command → LLM → Full Plan → Execute All
```
- Simplest architecture
- No adaptation to execution outcomes
- Best for predictable environments

**2. Closed-Loop Planning**: Monitor execution, re-plan on failures
```
Command → LLM → Plan → Execute Step → Check → [Next Step | Re-plan]
```
- Handles failures gracefully
- More robust in uncertain environments
- Higher API costs (more LLM calls)

**3. Hierarchical Planning**: Multiple LLM levels for different abstraction
```
Command → Strategic LLM (high-level) → Tactical LLM (mid-level) → Execution
```
- Separates "what to do" from "how to do it"
- Complex to implement
- Best for very complex tasks

For most robotics applications, **closed-loop planning** provides the best balance of robustness and complexity.

## Practical Example: LLM Task Planner

Let's build a complete cognitive planning system.

### LLM Planner Implementation

```python
#!/usr/bin/env python3
"""LLM Task Planner - Decompose commands into action sequences."""

import json
from openai import OpenAI
from typing import List, Dict, Optional

class LLMPlanner:
    """LLM-based task planner for robot action sequences."""

    def __init__(self, model: str = "gpt-4"):
        self.client = OpenAI()
        self.model = model
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        """Build system prompt with few-shot examples."""
        return """You are a robot task planner. Convert commands to action sequences.

Available actions:
- navigate: Move to location (params: location_name)
- detect: Find object (params: object_type, color)
- grasp: Pick up object (params: gripper_force)
- place: Put down object (params: target_location)
- speak: Say message (params: message)
- scan: Survey area (params: scan_type)

Output JSON format:
{
    "goal": "<command>",
    "plan": [
        {"step": 1, "type": "<action>", "target": "<target>",
         "parameters": {...}, "description": "<desc>",
         "estimated_duration": <seconds>}
    ],
    "total_estimated_time": <seconds>,
    "confidence": <0.0-1.0>
}

Examples:

Command: "Pick up the red cup from the table"
{
    "goal": "Pick up the red cup from the table",
    "plan": [
        {"step": 1, "type": "navigate", "target": "table",
         "parameters": {"location_name": "table"},
         "description": "Move to the table", "estimated_duration": 10.0},
        {"step": 2, "type": "detect", "target": "red cup",
         "parameters": {"object_type": "cup", "color": "red"},
         "description": "Locate the red cup", "estimated_duration": 3.0},
        {"step": 3, "type": "grasp", "target": "red cup",
         "parameters": {"gripper_force": 0.5},
         "description": "Grasp the red cup", "estimated_duration": 5.0}
    ],
    "total_estimated_time": 18.0,
    "confidence": 0.95
}

Command: "Go to the kitchen"
{
    "goal": "Go to the kitchen",
    "plan": [
        {"step": 1, "type": "navigate", "target": "kitchen",
         "parameters": {"location_name": "kitchen"},
         "description": "Navigate to kitchen", "estimated_duration": 15.0}
    ],
    "total_estimated_time": 15.0,
    "confidence": 0.98
}"""

    def plan_task(self, command: str) -> Optional[Dict]:
        """Generate action plan from command."""
        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": command}
            ],
            temperature=0.0,  # Deterministic for safety
            max_tokens=1000
        )

        # Parse JSON response
        text = response.choices[0].message.content.strip()

        # Handle markdown code blocks
        if "```json" in text:
            text = text.split("```json")[1].split("```")[0]

        plan = json.loads(text)

        # Validate structure
        if self._validate_plan(plan):
            return plan
        return None

    def _validate_plan(self, plan: Dict) -> bool:
        """Validate plan structure."""
        required = ['goal', 'plan', 'confidence']
        if not all(k in plan for k in required):
            return False
        if not plan['plan']:  # Empty plan
            return False
        return True
```

**Key Design Decisions**:

**Few-Shot Examples**: The system prompt includes complete examples showing:
- Simple commands ("Go to kitchen" → 1 action)
- Complex commands ("Pick up cup" → 3 actions)
- All relevant parameters for each action type

**JSON Output**: Structured output enables reliable parsing. The format includes:
- `step`: Ordering for sequential execution
- `type`: Action primitive (navigate, detect, etc.)
- `target`: What the action operates on
- `parameters`: Action-specific configuration
- `estimated_duration`: For progress tracking

**Validation**: Even with careful prompting, LLMs can produce malformed output. The `_validate_plan` method ensures the plan is executable.

### Action Executor

```python
#!/usr/bin/env python3
"""Action Executor - Execute plans with progress tracking."""

import time
from typing import Dict, List, Callable
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ExecutionStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"

@dataclass
class ActionResult:
    action_type: str
    status: ExecutionStatus
    message: str
    duration: float

class ActionExecutor(Node):
    """Execute action sequences with progress tracking."""

    def __init__(self):
        super().__init__('action_executor')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/execution_status', 10)

        # Action handlers
        self.handlers = {
            'navigate': self._execute_navigate,
            'detect': self._execute_detect,
            'grasp': self._execute_grasp,
            'place': self._execute_place,
            'speak': self._execute_speak,
        }

        self.progress_callback = None

    def execute_plan(self, plan: Dict) -> List[ActionResult]:
        """Execute all actions in plan sequentially."""
        results = []
        actions = plan.get('plan', [])
        total = len(actions)

        for i, action in enumerate(actions):
            step = i + 1
            self._report_progress(step, total, "IN_PROGRESS",
                                 action.get('description', ''))

            start = time.time()
            result = self._execute_action(action)
            result.duration = time.time() - start

            results.append(result)

            if result.status == ExecutionStatus.FAILED:
                self._report_progress(step, total, "FAILED", result.message)
                break  # Stop on failure

            self._report_progress(step, total, "COMPLETED",
                                 f"Completed in {result.duration:.1f}s")

        return results

    def _execute_action(self, action: Dict) -> ActionResult:
        """Execute single action."""
        action_type = action.get('type', 'unknown')
        handler = self.handlers.get(action_type)

        if handler is None:
            return ActionResult(action_type, ExecutionStatus.FAILED,
                              f"Unknown action: {action_type}", 0.0)

        return handler(action)

    def _execute_navigate(self, action: Dict) -> ActionResult:
        """Execute navigation action."""
        target = action.get('target', 'unknown')

        # In real system: call Nav2 action client
        # For demo: simulate with velocity commands
        twist = Twist()
        twist.linear.x = 0.3
        self.cmd_vel_pub.publish(twist)
        time.sleep(2.0)  # Simulate travel
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        return ActionResult('navigate', ExecutionStatus.COMPLETED,
                          f"Arrived at {target}", 0.0)

    def _execute_detect(self, action: Dict) -> ActionResult:
        """Execute detection action."""
        target = action.get('target', 'object')

        # In real system: call vision service
        # For demo: simulate detection
        time.sleep(1.0)

        return ActionResult('detect', ExecutionStatus.COMPLETED,
                          f"Found {target}", 0.0,
                          {'position': [1.0, 0.5, 0.8]})

    def _execute_grasp(self, action: Dict) -> ActionResult:
        """Execute grasp action."""
        target = action.get('target', 'object')

        # In real system: call MoveIt2 action client
        # For demo: simulate grasping
        time.sleep(2.0)

        return ActionResult('grasp', ExecutionStatus.COMPLETED,
                          f"Grasped {target}", 0.0)

    def _execute_place(self, action: Dict) -> ActionResult:
        """Execute place action."""
        target = action.get('target', 'location')
        time.sleep(1.5)
        return ActionResult('place', ExecutionStatus.COMPLETED,
                          f"Placed at {target}", 0.0)

    def _execute_speak(self, action: Dict) -> ActionResult:
        """Execute speak action."""
        message = action.get('parameters', {}).get('message', '')
        # Publish to speech topic
        msg = String()
        msg.data = message
        # In real system: text-to-speech node subscribes
        return ActionResult('speak', ExecutionStatus.COMPLETED,
                          f"Said: {message}", 0.0)

    def _report_progress(self, step, total, status, message):
        """Report execution progress."""
        status_msg = String()
        status_msg.data = f"Step {step}/{total}: {status} - {message}"
        self.status_pub.publish(status_msg)
        self.get_logger().info(status_msg.data)
```

**Progress Tracking**: The executor publishes status updates to `/execution_status`, enabling UI integration or logging systems to monitor progress.

**Action Handlers**: Each action type has a dedicated handler. In production, these would call real services:
- `navigate` → Nav2 `NavigateToPose` action (see [Module 3, Nav2 Basics](../module-03-isaac-navigation/06-nav2-basics.md))
- `detect` → Vision service (YOLO, OpenCV)
- `grasp` → MoveIt2 `PickPlace` action

### Error Handler with Re-Planning

```python
#!/usr/bin/env python3
"""Error Handler - Recover from failures using LLM re-planning."""

import json
from typing import Dict, Optional
from enum import Enum
from openai import OpenAI

class ErrorType(Enum):
    NAVIGATION_BLOCKED = "navigation_blocked"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"

class ErrorHandler:
    """Handle execution errors with LLM-powered recovery."""

    def __init__(self, model: str = "gpt-4"):
        self.client = OpenAI()
        self.model = model
        self.max_retries = 3

    def handle_error(self, error_type: ErrorType,
                    failed_action: Dict,
                    original_plan: Dict,
                    attempt: int) -> Optional[Dict]:
        """Generate recovery plan for error."""

        # Simple retry for transient failures
        if error_type == ErrorType.GRASP_FAILED and attempt < self.max_retries:
            return self._retry_with_adjustment(failed_action)

        # LLM re-planning for complex failures
        if error_type in [ErrorType.NAVIGATION_BLOCKED, ErrorType.OBJECT_NOT_FOUND]:
            return self._replan_with_llm(error_type, failed_action, original_plan)

        return None  # Unrecoverable

    def _retry_with_adjustment(self, action: Dict) -> Dict:
        """Retry action with adjusted parameters."""
        adjusted = action.copy()
        params = adjusted.get('parameters', {})

        # Increase gripper force for grasp retry
        if action['type'] == 'grasp':
            params['gripper_force'] = min(1.0, params.get('gripper_force', 0.5) + 0.2)

        adjusted['parameters'] = params
        adjusted['description'] = f"Retry: {action.get('description', '')}"

        return {"plan": [adjusted], "is_retry": True}

    def _replan_with_llm(self, error_type: ErrorType,
                        failed_action: Dict,
                        original_plan: Dict) -> Optional[Dict]:
        """Use LLM to generate alternative approach."""

        prompt = f"""The robot encountered an error.

Original Goal: {original_plan.get('goal')}
Failed Action: {json.dumps(failed_action)}
Error: {error_type.value}

Generate an alternative approach avoiding the failed action.
Output JSON with 'new_actions' list and 'explanation'."""

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "You are a robot recovery planner."},
                {"role": "user", "content": prompt}
            ],
            temperature=0.2,
            max_tokens=500
        )

        text = response.choices[0].message.content
        if "```json" in text:
            text = text.split("```json")[1].split("```")[0]

        recovery = json.loads(text)
        return {"plan": recovery.get('new_actions', []),
                "explanation": recovery.get('explanation', '')}
```

**Recovery Strategies**:
- **Retry**: For transient failures (grasp slip), retry with adjusted parameters
- **Re-plan**: For environmental changes (blocked path, missing object), ask LLM for alternative

**Context Passing**: The re-planning prompt includes the original goal, failed action, and error type. The LLM generates alternatives that avoid the failure mode.

## Integration with ROS 2

This chapter builds on ROS 2 concepts from Module 1:

**Actions** (from [Module 1, Chapter 3](../module-01-robotic-nervous-system/03-services-and-actions.md)): Long-running tasks like navigation use ROS 2 actions, which provide:
- Feedback during execution (progress updates)
- Goal preemption (cancel navigation if needed)
- Result reporting (success/failure)

**Navigation Integration**: In production, the `navigate` action would use Nav2:
```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

# Send goal
goal = NavigateToPose.Goal()
goal.pose.pose.position.x = target_x
goal.pose.pose.position.y = target_y
future = self.nav_client.send_goal_async(goal)
```

**Manipulation Integration**: Grasp actions would use MoveIt2:
```python
from moveit_msgs.action import MoveGroup
# Configure grasp pose, call action client
```

## Common Pitfalls and Troubleshooting

### Issue 1: LLM Generates Invalid Actions

**Symptom**: Plan contains action types not in the robot's vocabulary

**Solution**:
```python
# Strict validation
VALID_ACTIONS = {'navigate', 'detect', 'grasp', 'place', 'speak', 'wait', 'scan'}

def _validate_plan(self, plan):
    for action in plan.get('plan', []):
        if action.get('type') not in VALID_ACTIONS:
            return False
    return True
```

**Prompt Engineering**: Explicitly list valid actions in system prompt and include examples of each.

### Issue 2: Actions Fail Silently

**Symptom**: Plan executes but robot doesn't move

**Cause**: ROS 2 topics not connected, simulation not running

**Solution**:
```bash
# Verify topics
ros2 topic list | grep cmd_vel
ros2 topic echo /cmd_vel  # Should show messages when executing

# Check node connections
ros2 node info /action_executor
```

### Issue 3: Re-planning Loops Forever

**Symptom**: Error → re-plan → same error → re-plan...

**Solution**:
```python
class ErrorHandler:
    def __init__(self):
        self.attempt_counts = {}  # Track per-goal attempts
        self.max_total_attempts = 5

    def handle_error(self, error_type, action, plan, attempt):
        goal = plan.get('goal', '')
        self.attempt_counts[goal] = self.attempt_counts.get(goal, 0) + 1

        if self.attempt_counts[goal] > self.max_total_attempts:
            return None  # Give up

        # ... normal handling
```

### Issue 4: High API Costs

**Symptom**: OpenAI bills higher than expected

**Solutions**:
```python
# 1. Use cheaper model for simple plans
model = "gpt-3.5-turbo"  # 10x cheaper than GPT-4

# 2. Cache common commands
self.plan_cache = {}
if command in self.plan_cache:
    return self.plan_cache[command]

# 3. Reduce max_tokens
response = self.client.chat.completions.create(
    max_tokens=500,  # Instead of 1000
    # ...
)
```

## Chapter Summary

**Key Takeaways**:

1. **Cognitive Planning**: LLMs decompose natural language goals into executable action sequences, replacing manual task specification with learned knowledge

2. **Few-Shot Prompting**: Include 3-5 diverse examples in the system prompt, covering simple and complex commands with all action types

3. **Structured Output**: Request JSON format for reliable parsing; validate structure before execution

4. **Closed-Loop Architecture**: Monitor execution, detect failures, and re-plan when needed for robust operation in uncertain environments

5. **Error Recovery**: Use retry for transient failures, LLM re-planning for environmental changes, and graceful degradation for unrecoverable errors

6. **ROS 2 Integration**: LLM plans map to ROS 2 actions (Nav2 for navigation, MoveIt2 for manipulation, vision services for detection)

**Critical Code Patterns**:

- **Plan generation**: `plan = planner.plan_task("Pick up the cup")`
- **Sequential execution**: `for action in plan['plan']: executor.execute(action)`
- **Error handling**: `recovery = handler.handle_error(error_type, action, plan, attempt)`
- **Progress tracking**: Publish status to `/execution_status` topic

**Common Pitfalls**:

- **Invalid actions**: Validate LLM output against known action types
- **Silent failures**: Monitor ROS 2 topics, check node connections
- **Re-planning loops**: Track total attempts, implement give-up threshold
- **API costs**: Cache plans, use cheaper models, reduce token limits

**Prerequisites for Next Chapter**:

- Understanding of LLM task decomposition and few-shot prompting
- Familiarity with action execution and error handling patterns
- Experience with ROS 2 publishers and action concepts
- Awareness of navigation and manipulation integration points

**Additional Resources**:

- [SayCan: Grounding Language in Robotic Affordances](https://say-can.github.io/)
- [Inner Monologue: Embodied Reasoning through Planning with Language Models](https://arxiv.org/abs/2207.05608)
- [ROS 2 Navigation (Nav2) Documentation](https://docs.nav2.org/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/)

---

**Next Chapter**: [Autonomous Humanoid Capstone](04-capstone-autonomous-humanoid.md) - Build a complete autonomous system integrating voice, planning, navigation, vision, and manipulation into a unified humanoid robot controller.
