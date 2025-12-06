---
title: "Large Language Models for Robotics"
sidebar_label: "LLMs for Robotics"
sidebar_position: 3
description: "Cognitive planning with LLMs - from natural language to robot actions"
tags: [llm, gpt, cognitive-planning, task-planning]
keywords: [Large Language Models, LLM robotics, GPT-4, cognitive planning, prompt engineering, function calling]
difficulty: intermediate
estimated_time: "2.5 hours"
prerequisites: ["Chapter 2: Voice-to-Action with Whisper"]
---

# Large Language Models for Robotics

*Content coming soon. This chapter will cover:*

## LLM Fundamentals

### What are Large Language Models?

- Transformer architecture
- Pre-training on massive text corpora
- Emergent capabilities at scale
- Few-shot and zero-shot learning
- In-context learning

### Popular LLMs for Robotics

| Model | Provider | Context | Strengths |
|-------|----------|---------|-----------|
| GPT-4 | OpenAI | 128K tokens | Best reasoning, function calling |
| GPT-4o | OpenAI | 128K tokens | Faster, cheaper, multimodal |
| Claude 3.5 Sonnet | Anthropic | 200K tokens | Long context, safety-focused |
| LLaMA 3 | Meta | 8K-128K | Open-source, self-hostable |
| Gemini Pro | Google | 1M tokens | Massive context window |
| Mistral | Mistral AI | 32K tokens | Open-source, efficient |

### Commercial vs Open-Source

**Commercial (API-based)**
- ✅ Best performance
- ✅ Easy to use
- ✅ Regularly updated
- ❌ Costs money
- ❌ Requires internet
- ❌ Data privacy concerns

**Open-Source (Self-hosted)**
- ✅ Free to use
- ✅ Full control
- ✅ Works offline
- ❌ Requires GPU
- ❌ Setup complexity
- ❌ Lower performance

## Why LLMs for Robotics?

### Traditional Approach

```python
# Hardcoded task planning
def clean_room():
    navigate_to("living_room")
    detect_objects("toys")
    for toy in detected_toys:
        grasp(toy)
        navigate_to("toy_box")
        place(toy)
    return_to("charging_station")
```

**Problems:**
- Brittle and inflexible
- Doesn't handle variations
- Requires explicit programming for every task

### LLM Approach

```python
# Natural language task specification
command = "Clean the room"
plan = llm.generate_plan(command, robot_capabilities, environment)
execute_plan(plan)
```

**Advantages:**
- Natural language interface
- Handles variations and edge cases
- Generalizes to new tasks
- Can reason about physical constraints

## Prompt Engineering for Robotics

### Basic Prompt Structure

```python
system_prompt = """
You are a robot assistant. You can perform these actions:
- navigate(location): Move to a location
- detect(object_type): Find objects
- grasp(object): Pick up an object
- place(location): Put down an object

Generate a step-by-step plan for the user's command.
Output as a JSON list of actions.
"""

user_command = "Bring me the red cup from the kitchen"
```

### Advanced Techniques

**1. Few-Shot Examples**

```python
examples = """
Example 1:
Command: "Put the book on the shelf"
Plan: [
  {"action": "navigate", "target": "book_location"},
  {"action": "detect", "object": "book"},
  {"action": "grasp", "object": "book"},
  {"action": "navigate", "target": "shelf"},
  {"action": "place", "location": "shelf"}
]

Example 2:
Command: "Clean the table"
Plan: [
  {"action": "navigate", "target": "table"},
  {"action": "detect", "object": "debris"},
  ...
]
"""
```

**2. Chain-of-Thought Reasoning**

```python
prompt = """
Think step-by-step about how to accomplish this task:

1. What objects are involved?
2. What is their current state?
3. What is the desired final state?
4. What actions are needed to get there?
5. Are there any safety concerns?

Command: "Stack the blocks"
"""
```

**3. Self-Critique and Refinement**

```python
# First generate a plan
plan = llm.generate_plan(command)

# Then critique it
critique_prompt = f"""
Review this plan for issues:
{plan}

Check for:
- Missing steps
- Unsafe actions
- Inefficiencies
- Physical impossibilities

Provide an improved plan.
"""
```

## Function Calling / Tool Use

### What is Function Calling?

LLMs can generate structured outputs that call specific functions:

```python
# LLM generates:
{
  "function": "navigate",
  "arguments": {
    "location": "kitchen",
    "speed": "normal"
  }
}

# Your code executes:
robot.navigate(location="kitchen", speed="normal")
```

### Defining Robot Capabilities

```python
tools = [
    {
        "name": "navigate",
        "description": "Move the robot to a specified location",
        "parameters": {
            "location": {"type": "string", "description": "Target location"},
            "speed": {"type": "string", "enum": ["slow", "normal", "fast"]}
        }
    },
    {
        "name": "grasp_object",
        "description": "Pick up an object",
        "parameters": {
            "object_id": {"type": "string"},
            "force": {"type": "number", "description": "Grip force in Newtons"}
        }
    }
]
```

### OpenAI Function Calling

```python
import openai

response = openai.ChatCompletion.create(
    model="gpt-4",
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": "Move to the kitchen"}
    ],
    functions=tools,
    function_call="auto"
)

# Extract function call
function_call = response.choices[0].message.function_call
function_name = function_call.name
arguments = json.loads(function_call.arguments)

# Execute
execute_robot_action(function_name, arguments)
```

## Grounding LLMs in Robot State

### Providing Context

```python
system_prompt = f"""
Current robot state:
- Location: {robot.current_location}
- Battery: {robot.battery_level}%
- Holding: {robot.gripper_contents}
- Detected objects: {robot.detected_objects}

Environment:
- Rooms: {list_of_rooms}
- Known object locations: {object_map}
"""
```

### Real-Time Feedback Loop

```
User Command → LLM Plan → Execute Action
                            ↓
                    Observe Outcome
                            ↓
                    Update Context
                            ↓
                    Next Action (back to LLM)
```

## Task and Motion Planning (TAMP) with LLMs

### High-Level Planning (LLM)

```python
# LLM generates high-level plan
plan = ["navigate(kitchen)", "detect(cup)", "grasp(cup)", "navigate(user)", "place(table)"]
```

### Low-Level Motion Planning (Traditional)

```python
# For each high-level action, use motion planner
for action in plan:
    if action.startswith("navigate"):
        path = nav2_planner.plan(current_pos, target_pos)
        execute_path(path)
    elif action.startswith("grasp"):
        moveit_plan = moveit.plan_grasp(object_pose)
        execute_moveit(moveit_plan)
```

### The Hybrid Approach

- LLM: Task-level reasoning ("what to do")
- Classical planners: Motion-level execution ("how to do it")
- Best of both worlds

## Handling Uncertainty and Failure

### Clarification Dialogues

```python
if command_is_ambiguous(user_input):
    clarification = llm.generate_question(user_input, context)
    # "Which cup? The red one or the blue one?"
    user_response = wait_for_response()
    refined_command = llm.refine_command(user_input, user_response)
```

### Recovery Behaviors

```python
try:
    execute_plan(plan)
except ActionFailed as e:
    recovery_plan = llm.generate_recovery_plan(
        original_plan=plan,
        failed_action=e.action,
        error_message=e.message,
        current_state=robot.state
    )
    execute_plan(recovery_plan)
```

## Safety and Validation

### Validating LLM Outputs

```python
def validate_action(action):
    # Check if action is in whitelist
    if action not in ALLOWED_ACTIONS:
        return False

    # Check parameters
    if action == "navigate":
        if target not in KNOWN_LOCATIONS:
            return False

    # Check safety constraints
    if action == "grasp" and force > MAX_FORCE:
        return False

    return True
```

### Guardrails

```python
FORBIDDEN_KEYWORDS = ["throw", "break", "damage", "harm"]

def check_safety(command):
    for keyword in FORBIDDEN_KEYWORDS:
        if keyword in command.lower():
            return False, f"Unsafe command: contains '{keyword}'"
    return True, "Safe"
```

## Integration with ROS 2

### LLM Planner Node

```python
class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner')
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10
        )
        self.action_pub = self.create_publisher(
            RobotAction, 'planned_actions', 10
        )
        self.llm = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))

    def command_callback(self, msg):
        plan = self.generate_plan(msg.data)
        for action in plan:
            self.action_pub.publish(action)
```

## Cost Considerations

### Token Usage

```python
# Estimate costs
tokens_per_request = 500  # prompt + response
cost_per_1k_tokens = 0.03  # GPT-4
requests_per_day = 100

daily_cost = (tokens_per_request / 1000) * cost_per_1k_tokens * requests_per_day
# = $1.50/day
```

### Optimization Strategies

1. **Cache common prompts**
2. **Use cheaper models when possible** (GPT-3.5 vs GPT-4)
3. **Batch requests**
4. **Local LLMs for simple tasks**
5. **Hybrid approach**: LLM for planning, rules for execution

## Open-Source LLMs

### Running LLaMA Locally

```bash
# Install Ollama
curl https://ollama.ai/install.sh | sh

# Download LLaMA 3
ollama pull llama3

# Run
ollama run llama3
```

### Integration with ROS 2

```python
import requests

def query_local_llm(prompt):
    response = requests.post('http://localhost:11434/api/generate', json={
        'model': 'llama3',
        'prompt': prompt
    })
    return response.json()['response']
```

## Hands-On Projects

### Project 1: Basic LLM Planning
- Send a command to GPT-4
- Get back a structured plan
- Validate the plan
- Print results

### Project 2: ROS 2 Integration
- Create LLM planner node
- Subscribe to voice commands
- Generate action plans
- Publish to action topic

### Project 3: Feedback Loop
- Execute actions
- Observe results
- Send feedback to LLM
- Replan if needed

### Project 4: Local LLM
- Set up Ollama
- Run LLaMA 3 locally
- Compare with GPT-4
- Evaluate trade-offs

## Best Practices

1. **Clear system prompts**: Define robot capabilities precisely
2. **Structured outputs**: Use JSON for action plans
3. **Validation**: Always validate LLM outputs before execution
4. **Feedback loops**: Keep LLM updated with robot state
5. **Graceful degradation**: Fall back to hardcoded behaviors on failure
6. **Cost monitoring**: Track API usage

*For now, explore [LangChain for Robotics](https://python.langchain.com/) and [OpenAI Function Calling](https://platform.openai.com/docs/guides/function-calling).*
