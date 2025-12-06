---
title: "Natural Language to Robot Actions"
sidebar_label: "NL to Actions"
sidebar_position: 4
description: "Translating high-level commands into executable robot action sequences"
tags: [natural-language, action-primitives, task-planning, semantic-parsing]
keywords: [natural language processing, action primitives, task planning, semantic parsing, TAMP]
difficulty: intermediate
estimated_time: "2 hours"
prerequisites: ["Chapter 3: LLMs for Robotics"]
---

# Natural Language to Robot Actions

*Content coming soon. This chapter will cover:*

## The Translation Challenge

### From Language to Execution

```
"Clean the room"
    ↓ (What does this mean?)
[navigate_to(room),
 detect_objects(clutter),
 for each object: grasp() → navigate_to(storage) → place(),
 return_to(start)]
```

**Key Challenges:**
- Ambiguity: What is "clean"? How clean?
- Grounding: Which room? What counts as clutter?
- Planning: What order? What if something fails?
- Safety: Don't break anything!

## Semantic Parsing

### What is Semantic Parsing?

Converting natural language to structured representations:

```
Input: "Put the red box on the table"
Output: {
  "action": "place",
  "object": {"type": "box", "color": "red"},
  "location": {"type": "surface", "name": "table"}
}
```

### Approaches

**1. Rule-Based Parsing**

```python
def parse_command(text):
    if "move" in text and "to" in text:
        match = re.search(r"move (.*) to (.*)", text)
        return {"action": "move", "object": match.group(1), "target": match.group(2)}
```

- ✅ Fast and deterministic
- ❌ Brittle, doesn't handle variations

**2. LLM-Based Parsing**

```python
prompt = f"""
Parse this command into structured JSON:
Command: "{user_input}"

Output format:
{{
  "action": "move|grasp|place|navigate",
  "target": "object or location",
  "modifiers": {{...}}
}}
"""
parsed = llm(prompt)
```

- ✅ Handles variations and ambiguity
- ❌ Slower, requires API

**3. Trained Semantic Parsers**

- Fine-tuned models (BERT, T5) for command→structure
- Trained on robotics command datasets
- Balance of speed and flexibility

## Action Primitive Libraries

### What are Action Primitives?

Low-level actions that robots can reliably execute:

```python
PRIMITIVES = {
    "navigate": lambda target: nav2.navigate_to(target),
    "detect": lambda obj_type: vision.detect_objects(obj_type),
    "grasp": lambda obj: manipulation.grasp_object(obj),
    "place": lambda loc: manipulation.place_at(loc),
    "wait": lambda duration: time.sleep(duration)
}
```

### Designing Good Primitives

**Characteristics:**
- **Atomic**: Can't be broken down further
- **Reliable**: High success rate
- **Composable**: Can be combined into complex behaviors
- **Parametrized**: Flexible through arguments

**Example Library:**

```python
# Navigation primitives
navigate_to(location: str)
navigate_to_pose(x: float, y: float, theta: float)
follow_path(waypoints: List[Pose])

# Perception primitives
detect_objects(object_type: str) -> List[DetectedObject]
track_object(object_id: str) -> ObjectTracker
estimate_pose(object_id: str) -> Pose6D

# Manipulation primitives
grasp_object(object_id: str, approach: str) -> bool
place_at_pose(pose: Pose) -> bool
open_gripper()
close_gripper(force: float)

# Utility primitives
wait(duration: float)
say(text: str)
confirm_with_user(question: str) -> bool
```

## Task and Motion Planning (TAMP)

### The Two-Level Hierarchy

**Task Level**: What to do (symbolic)
```
["get(cup)", "goto(user)", "give(cup)"]
```

**Motion Level**: How to do it (geometric)
```
[nav2_path, moveit_trajectory, grasp_pose]
```

### TAMP with LLMs

**LLM handles task planning:**

```python
command = "Bring me a coffee"
task_plan = llm.plan(command, available_actions=PRIMITIVES.keys())
# ["navigate(kitchen)", "detect(coffee)", "grasp(coffee)", "navigate(user)", "place(table)"]
```

**Classical planners handle motion:**

```python
for task in task_plan:
    if task.action == "navigate":
        motion_plan = nav2_planner.plan(current, task.target)
        execute(motion_plan)
    elif task.action == "grasp":
        grasp_plan = moveit.plan_grasp(task.object_pose)
        execute(grasp_plan)
```

### Handling Failures

```python
def execute_with_retry(task_plan):
    for i, task in enumerate(task_plan):
        success = execute_primitive(task)
        if not success:
            # Replan from current state
            remaining_goal = extract_goal(task_plan[i:])
            new_plan = llm.replan(remaining_goal, current_state)
            return execute_with_retry(new_plan)
```

## Grounding Language in Perception

### The Grounding Problem

```
"Pick up the red box"
```

- Which object in the scene is "the red box"?
- Need to connect language to visual percepts

### Visual Grounding Pipeline

```
1. Perceive: Detect all objects in scene
2. Filter: Apply language constraints (color, shape, size)
3. Resolve: Handle ambiguity (if multiple matches)
4. Return: Object ID/pose for manipulation
```

### Implementation

```python
def ground_object(description: str) -> DetectedObject:
    # Parse description
    attributes = parse_description(description)  # {"color": "red", "type": "box"}

    # Get all detected objects
    objects = perception.detect_all()

    # Filter by attributes
    candidates = [
        obj for obj in objects
        if match_attributes(obj, attributes)
    ]

    # Resolve ambiguity
    if len(candidates) == 0:
        raise ObjectNotFoundError(description)
    elif len(candidates) > 1:
        # Ask for clarification or use spatial reasoning
        selected = resolve_ambiguity(candidates, context)
    else:
        selected = candidates[0]

    return selected
```

### Spatial Reasoning

```
"The cup on the left"
"The box next to the laptop"
"The tallest bottle"
```

- Requires understanding spatial relationships
- LLMs or specialized models (CLIP, Grounding DINO)

## Handling Ambiguity

### Types of Ambiguity

**1. Referential Ambiguity**
```
"Pick up the box"  # Which box? Multiple boxes present
```

**2. Action Ambiguity**
```
"Clean the table"  # Wipe it? Remove objects? Both?
```

**3. Parameter Ambiguity**
```
"Move forward"  # How far? How fast?
```

### Resolution Strategies

**1. Ask for Clarification**

```python
if len(candidates) > 1:
    question = f"I see {len(candidates)} boxes. Which one?"
    # Present options or ask user to point
```

**2. Use Context and Defaults**

```python
if "forward" in command and distance is None:
    distance = DEFAULT_MOVE_DISTANCE  # e.g., 1 meter
```

**3. Utilize Environmental Cues**

```python
# If user says "that box", check where they're looking/pointing
gaze_direction = head_pose_estimator.get_gaze()
closest_to_gaze = find_closest_object(gaze_direction, candidates)
```

## Command Validation

### Pre-Execution Checks

```python
def validate_command(parsed_command):
    checks = [
        check_safety(parsed_command),
        check_feasibility(parsed_command),
        check_permissions(parsed_command)
    ]
    return all(checks)

def check_safety(cmd):
    # Don't grasp dangerous objects
    if cmd["object"] in DANGEROUS_OBJECTS:
        return False

    # Don't navigate near hazards
    if cmd["action"] == "navigate" and is_near_hazard(cmd["target"]):
        return False

    return True

def check_feasibility(cmd):
    # Can we reach the object?
    if cmd["action"] == "grasp":
        return is_reachable(cmd["object_pose"])

    # Is the location navigable?
    if cmd["action"] == "navigate":
        return is_navigable(cmd["target"])

    return True
```

### Execution Monitoring

```python
def execute_with_monitoring(action):
    start_time = time.time()

    # Execute action
    result = execute_primitive(action)

    # Check timeout
    if time.time() - start_time > action.timeout:
        raise TimeoutError(action)

    # Verify outcome
    if not verify_action_success(action, result):
        raise ActionFailedError(action)

    return result
```

## Composing Complex Behaviors

### Sequential Composition

```python
# "Get the cup and bring it to me"
plan = [
    navigate_to("kitchen"),
    detect("cup"),
    grasp("cup"),
    navigate_to("user"),
    place_on("table")
]
```

### Conditional Composition

```python
# "If the door is closed, open it, then enter"
plan = [
    navigate_to("door"),
    detect("door"),
    if_closed(
        then=[grasp("handle"), pull(), wait(1)],
        else_=[]
    ),
    navigate_through("door")
]
```

### Parallel Composition

```python
# "Look for the cup while moving to the kitchen"
plan = parallel(
    [navigate_to("kitchen")],
    [detect_continuously("cup")]
)
```

## Dialogue and Interaction

### Multi-Turn Commands

```
User: "Bring me a drink"
Robot: "What kind of drink?"
User: "Coffee"
Robot: "How do you take it?"
User: "Black, no sugar"
Robot: [executes plan]
```

### Maintaining Context

```python
class DialogueManager:
    def __init__(self):
        self.conversation_history = []
        self.current_goal = None

    def process_utterance(self, text):
        # Add to history
        self.conversation_history.append({"role": "user", "content": text})

        # Update goal with context
        response = llm.chat(self.conversation_history + system_prompt)

        # Extract action
        if is_clarification_question(response):
            return ask_user(response)
        else:
            action_plan = parse_action_plan(response)
            return action_plan
```

## Example: Complete Pipeline

### Command: "Clean the room"

```python
# 1. Semantic Parsing
parsed = {
    "intent": "clean",
    "target": "room",
    "room_id": infer_current_room()
}

# 2. Task Planning (LLM)
task_plan = llm.plan(f"Clean room {parsed['room_id']}")
# Returns: ["navigate(room)", "detect(clutter)", "for_each_object(grasp, navigate(storage), place)", "return(start)"]

# 3. Grounding
clutter_objects = perception.detect_objects(category="clutter")

# 4. Execution
for obj in clutter_objects:
    grasp_success = execute_primitive("grasp", obj)
    if grasp_success:
        navigate_success = execute_primitive("navigate", "storage")
        place_success = execute_primitive("place", "storage_location")

# 5. Verification
final_clutter = perception.detect_objects(category="clutter")
if len(final_clutter) < initial_clutter_count * 0.2:
    robot.say("Room cleaned!")
else:
    robot.say("Room partially cleaned. Some objects could not be moved.")
```

## Hands-On Projects

### Project 1: Command Parser
- Build a rule-based parser for basic commands
- Extend with LLM-based parsing
- Compare robustness

### Project 2: Action Primitive Library
- Define 10+ primitives for your robot
- Implement ROS 2 wrappers
- Test each primitive individually

### Project 3: Visual Grounding
- Detect objects in a scene
- Ground language descriptions to objects
- Handle ambiguous references

### Project 4: Complete NL→Action Pipeline
- Voice input → Semantic parsing → Task planning → Execution
- Test with complex multi-step commands
- Implement error recovery

*For now, study [Task and Motion Planning](https://arxiv.org/abs/2010.01083) and explore [Grounding DINO](https://github.com/IDEA-Research/GroundingDINO).*
