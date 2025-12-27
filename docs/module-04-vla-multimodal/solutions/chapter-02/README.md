# Chapter 2: Voice-to-Action with Whisper

This directory contains the solution code for Chapter 2, demonstrating how to integrate OpenAI's Whisper speech recognition with ROS 2 for voice-controlled robot commands.

## Prerequisites

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS (recommended)
- **ROS 2**: Humble Hawksbill or newer
- **Python**: 3.8 or newer
- **Microphone**: USB or built-in microphone with good audio quality
- **GPU** (optional): NVIDIA GPU with CUDA for faster Whisper inference

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

### Whisper Installation Notes
Whisper requires PyTorch. For optimal performance:

**CPU Only**:
```bash
pip install openai-whisper
```

**GPU Acceleration (NVIDIA)**:
```bash
# Install PyTorch with CUDA first
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
# Then install Whisper
pip install openai-whisper
```

### Audio Setup
Verify your microphone is detected:
```bash
# List audio devices
python3 -c "import sounddevice; sounddevice.query_devices()"

# Test microphone
python3 audio_capture.py
```

## Files Overview

| File | Description |
|------|-------------|
| `whisper_voice_node.py` | Main ROS 2 node integrating Whisper with voice command publishing |
| `audio_capture.py` | Audio device enumeration, selection, and capture utilities |
| `command_publisher.py` | Natural language command parsing and Twist message publishing |
| `requirements.txt` | Python package dependencies |

## Usage

### Running the Voice Control Node

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Run the Whisper voice node
python3 whisper_voice_node.py
```

### Supported Voice Commands

The system recognizes the following commands:

| Command | Robot Action |
|---------|--------------|
| "move forward" / "go forward" / "forward" | Move forward at 0.5 m/s |
| "move backward" / "go back" / "backward" | Move backward at 0.5 m/s |
| "turn left" / "left" | Rotate left at 0.5 rad/s |
| "turn right" / "right" | Rotate right at -0.5 rad/s |
| "stop" / "halt" | Stop all movement |

### Speed Modifiers

Add speed modifiers to commands:
- "move forward slowly" → 0.2 m/s
- "turn left fast" → 0.8 rad/s

### Monitoring Output

In a separate terminal, monitor the published commands:
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Monitor velocity commands
ros2 topic echo /cmd_vel

# Monitor voice transcriptions
ros2 topic echo /voice_command
```

## Expected Output

```
============================================================
Whisper Voice Control Node
============================================================

Loading Whisper model: base
Whisper model loaded successfully
Voice command publisher created on /voice_command
Velocity publisher created on /cmd_vel
Starting continuous voice recognition...
Speak commands: 'forward', 'backward', 'left', 'right', 'stop'
Press Ctrl+C to exit

Recording 3.0s audio chunk...
Captured 48000 audio samples
Transcribed: 'move forward' (confidence: 0.92)
Matched keyword 'move forward' in 'move forward'
Published: 'move forward' -> linear=0.5, angular=0.0
```

## Troubleshooting

### Error: "No module named 'whisper'"
**Solution**: Install Whisper:
```bash
pip install openai-whisper
```

### Error: "No module named 'sounddevice'"
**Solution**: Install sounddevice:
```bash
pip install sounddevice
```

### Error: "No input devices found"
**Solution**: Check microphone connection:
```bash
# List available audio devices
arecord -l  # Linux
# Or
python3 -c "import sounddevice; print(sounddevice.query_devices())"
```

### Low Transcription Accuracy
**Solutions**:
1. Use a better microphone or reduce background noise
2. Speak clearly and at moderate speed
3. Use a larger Whisper model:
```python
# In whisper_voice_node.py, change parameter
self.declare_parameter('model_size', 'small')  # or 'medium', 'large'
```

### Slow Transcription
**Solutions**:
1. Use GPU acceleration (see installation notes)
2. Use a smaller model ('tiny' or 'base')
3. Reduce chunk duration:
```python
self.declare_parameter('chunk_duration', 2.0)  # 2 seconds instead of 3
```

### Commands Not Recognized
**Solutions**:
1. Check the transcription output - the command may be misheard
2. Add alternative phrasings to the command map in `whisper_voice_node.py`
3. Lower the confidence threshold:
```python
self.declare_parameter('confidence_threshold', 0.5)  # Default is 0.7
```

## Whisper Model Comparison

| Model | Size | Speed | Accuracy | VRAM |
|-------|------|-------|----------|------|
| tiny | 39M | Fast | Basic | ~1GB |
| base | 74M | Fast | Good | ~1GB |
| small | 244M | Medium | Better | ~2GB |
| medium | 769M | Slow | Very Good | ~5GB |
| large | 1.5GB | Very Slow | Best | ~10GB |

For robotics, `base` or `small` provides the best balance of speed and accuracy.

## Cost Considerations

Whisper runs locally, so there are **no API costs**. However, consider:
- **Compute**: GPU recommended for real-time performance
- **Memory**: Model size affects RAM/VRAM usage
- **Storage**: Models are downloaded once (~74MB for base)

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Microphone    │────>│ WhisperVoiceNode │────>│   /cmd_vel      │
│                 │     │                  │     │   (Twist)       │
└─────────────────┘     │  1. Capture      │     └─────────────────┘
                        │  2. Transcribe   │              │
                        │  3. Parse        │              v
                        │  4. Publish      │     ┌─────────────────┐
                        └──────────────────┘     │     Robot       │
                                │                │   Controller    │
                                v                └─────────────────┘
                        ┌──────────────────┐
                        │  /voice_command  │
                        │    (String)      │
                        └──────────────────┘
```

## Next Steps

After understanding voice control:
1. **Chapter 3**: Learn cognitive planning for multi-step tasks
2. **Chapter 4**: Build a complete autonomous humanoid system integrating voice, planning, and navigation

## Related Documentation

- [OpenAI Whisper GitHub](https://github.com/openai/whisper)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [sounddevice Documentation](https://python-sounddevice.readthedocs.io/)
- [geometry_msgs/Twist](https://docs.ros2.org/latest/api/geometry_msgs/msg/Twist.html)
