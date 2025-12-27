# Chapter 1: Basic LLM-ROS 2 Integration

This directory contains the solution code for Chapter 1, demonstrating how to integrate Large Language Models with ROS 2 for natural language robot control.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended)
- **ROS 2**: Humble Hawksbill or newer
- **Python**: 3.8 or newer
- **OpenAI API Key**: Required for LLM integration

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

## Setup

### 1. Source ROS 2 Environment
```bash
source /opt/ros/humble/setup.bash
```

### 2. Set OpenAI API Key
You'll need an OpenAI API key to use the LLM functionality.

**Option A: Environment Variable** (temporary):
```bash
export OPENAI_API_KEY="your-api-key-here"
```

**Option B: .env File** (persistent):
Create a `.env` file in this directory:
```bash
echo "OPENAI_API_KEY=your-api-key-here" > .env
```

**Get an API key**: https://platform.openai.com/api-keys

## Usage

### Running the Example
```bash
# Make sure ROS 2 is sourced and API key is set
python3 basic_llm_ros2.py
```

### Expected Output
The script will test several natural language commands:
```
[Test 1/4] Command: "Move forward 1 meter"
✓ Command processed successfully
Published command - linear.x: 0.5, angular.z: 0.0

[Test 2/4] Command: "Turn left 90 degrees"
✓ Command processed successfully
Published command - linear.x: 0.0, angular.z: 0.5
...
```

### Verifying ROS 2 Messages
In a separate terminal, you can monitor the published messages:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Listen to the /cmd_vel topic
ros2 topic echo /cmd_vel
```

## How It Works

1. **LLM Integration**: The node uses OpenAI's GPT-4 to interpret natural language commands
2. **Prompt Engineering**: A system prompt instructs the LLM to output structured JSON with robot parameters
3. **ROS 2 Publishing**: Parsed commands are published as `Twist` messages to the `/cmd_vel` topic
4. **Error Handling**: Network errors, rate limits, and invalid responses are handled gracefully

## Example Commands

Try these natural language commands:
- "Move forward 1 meter"
- "Turn left 90 degrees"
- "Move backward slowly"
- "Rotate right"
- "Stop the robot"

## Troubleshooting

### Error: "OPENAI_API_KEY environment variable not set"
**Solution**: Set your API key using one of the methods in the Setup section.

### Error: "ModuleNotFoundError: No module named 'rclpy'"
**Solution**: Make sure ROS 2 Humble is installed and sourced:
```bash
source /opt/ros/humble/setup.bash
```

### Error: "Authentication failed"
**Solution**: Check that your OpenAI API key is valid and has sufficient credits.

### Error: "API rate limit exceeded"
**Solution**: Wait a few moments and try again. Consider upgrading your OpenAI API plan if this happens frequently.

## Cost Considerations

This example uses OpenAI's GPT-4 API, which has associated costs:
- **Estimated cost per command**: ~$0.001-0.003 USD
- **Testing session (4 commands)**: ~$0.01 USD

For cost-free alternatives, consider:
- Using a local LLM (Llama 3, Mistral) with Ollama
- GPT-3.5-turbo (cheaper but less capable)

## Next Steps

After understanding this basic integration:
1. **Chapter 2**: Learn to add voice input with OpenAI Whisper
2. **Chapter 3**: Implement cognitive planning for multi-step tasks
3. **Chapter 4**: Build a complete autonomous humanoid system

## Related Documentation

- [OpenAI API Documentation](https://platform.openai.com/docs/api-reference)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
