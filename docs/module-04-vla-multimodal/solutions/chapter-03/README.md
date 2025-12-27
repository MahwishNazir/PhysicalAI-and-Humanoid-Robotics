# Chapter 3: Cognitive Planning with LLMs

This directory contains the solution code for Chapter 3, demonstrating how to use Large Language Models for cognitive task planning and execution in robotics.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended)
- **ROS 2**: Humble Hawksbill or newer
- **Python**: 3.8 or newer
- **OpenAI API Key**: Required for LLM planning

### ROS 2 Installation
If you don't have ROS 2 Humble installed:
```bash
# Follow the official installation guide
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Quick install (Ubuntu 22.04):
sudo apt update && sudo apt install -y ros-humble-desktop
```

### Python Dependencies
Install the required Python packages:
```bash
pip install -r requirements.txt
```

### OpenAI API Key Setup
Set your API key:
```bash
export OPENAI_API_KEY="your-api-key-here"
```

Or create a `.env` file:
```
OPENAI_API_KEY=your-api-key-here
```

## Files Overview

| File | Description |
|------|-------------|
| `llm_planner.py` | LLM-based task planner with few-shot prompting |
| `action_executor.py` | ROS 2 action executor with progress tracking |
| `error_handler.py` | Error classification and LLM-powered re-planning |
| `requirements.txt` | Python package dependencies |

## Usage

### Running the LLM Planner Demo

```bash
# 1. Set API key
export OPENAI_API_KEY="your-key"

# 2. Run the planner demo
python3 llm_planner.py
```

**Example Output**:
```
LLM Task Planner Demo
============================================================

Command: "Pick up the red cup from the table"
--------------------------------------------------
Goal: Pick up the red cup from the table
Estimated time: 18.0s
Confidence: 95%

Steps:
  1. Move to the table (10.0s)
  2. Locate the red cup on the table (3.0s)
  3. Grasp the red cup (5.0s)
```

### Running the Action Executor

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Run the executor demo
python3 action_executor.py
```

### Running the Error Handler Demo

```bash
# Run error recovery demo
python3 error_handler.py
```

## Supported Actions

The planner generates sequences using these action types:

| Action Type | Description | Parameters |
|-------------|-------------|------------|
| `navigate` | Move to location | `location_name`, `x`, `y`, `z` |
| `detect` | Find object | `object_type`, `color`, `size` |
| `grasp` | Pick up object | `gripper_force`, `approach_direction` |
| `place` | Put down object | `target_location`, `orientation` |
| `speak` | Say message | `message` |
| `wait` | Pause execution | `duration_seconds` |
| `scan` | Survey area | `scan_type` |

## Example Commands

Try these natural language commands:

- "Pick up the red cup from the table"
- "Go to the kitchen"
- "Clean the room"
- "Find and bring me my keys"
- "Put the book on the shelf"

## Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│  Natural        │     │   LLMPlanner    │     │ ActionExecutor  │
│  Language   ────┼────>│                 │────>│                 │
│  Command        │     │  Few-shot       │     │  Sequential     │
└─────────────────┘     │  Prompting      │     │  Execution      │
                        └────────┬────────┘     └────────┬────────┘
                                 │                       │
                                 v                       v
                        ┌─────────────────┐     ┌─────────────────┐
                        │  JSON Action    │     │   ROS 2         │
                        │  Sequence       │     │   Actions       │
                        └─────────────────┘     └────────┬────────┘
                                                         │
                                 ┌───────────────────────┘
                                 │
                                 v
                        ┌─────────────────┐
                        │  ErrorHandler   │
                        │                 │
                        │  Re-plan on     │
                        │  Failure        │
                        └─────────────────┘
```

## Error Recovery

The error handler supports these recovery strategies:

| Error Type | Default Strategy |
|------------|------------------|
| `navigation_blocked` | Re-plan with alternate route |
| `object_not_found` | Re-plan with expanded search |
| `grasp_failed` | Retry with adjusted parameters |
| `place_failed` | Retry |
| `timeout` | Retry |
| `hardware_error` | Request human help |

## Troubleshooting

### Error: "OPENAI_API_KEY environment variable not set"
**Solution**: Set your API key:
```bash
export OPENAI_API_KEY="your-key"
```

### Error: "JSON parsing error"
**Solution**: The LLM sometimes outputs invalid JSON. The code handles most cases, but you may need to:
- Lower the temperature parameter
- Use a more capable model (GPT-4 instead of GPT-3.5)

### Slow Planning
**Cause**: Network latency and LLM inference time

**Solutions**:
- Use GPT-3.5-turbo for faster response
- Cache common plans locally
- Reduce max_tokens in API call

### Actions Not Executing
**Cause**: ROS 2 not properly initialized

**Solution**:
```bash
source /opt/ros/humble/setup.bash
```

## Cost Considerations

LLM planning uses OpenAI API:
- **GPT-4**: ~$0.03-0.06 per plan
- **GPT-3.5-turbo**: ~$0.002-0.004 per plan

For production, consider:
- Caching common command→plan mappings
- Using GPT-3.5 for simple commands
- Implementing a local fallback planner

## Next Steps

After understanding cognitive planning:
1. **Chapter 4**: Build the complete autonomous humanoid capstone
2. Integrate with Nav2 for real navigation
3. Add computer vision for actual object detection
4. Connect to MoveIt2 for manipulation

## Related Documentation

- [OpenAI API Reference](https://platform.openai.com/docs/api-reference)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html)
- [Nav2 Documentation](https://docs.nav2.org/)
