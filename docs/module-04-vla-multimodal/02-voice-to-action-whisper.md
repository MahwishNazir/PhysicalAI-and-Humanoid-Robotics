# Chapter 2: Voice-to-Action with OpenAI Whisper

## Introduction

Voice is humanity's most natural interface. We speak to each other, to our devices, and increasingly, to our robots. While the previous chapter demonstrated text-based LLM control, typing commands is impractical when your hands are occupied, you're across the room, or you simply want a more intuitive interaction. Voice-controlled robots transform human-robot interaction from a technical exercise into a natural conversation.

OpenAI's Whisper represents a breakthrough in speech recognition. Unlike previous systems that required internet connectivity or extensive training on specific voices, Whisper runs locally and achieves near-human transcription accuracy across multiple languages and accents. For robotics, this means reliable voice recognition even in noisy environments like factories or outdoors, without sending audio to the cloud.

The implications for robotics are profound. Assembly line workers can verbally instruct collaborative robots while their hands remain on their tasks. Elderly individuals can command assistive robots without navigating complex interfaces. Field robots can receive instructions from operators wearing gloves or protective gear. Voice transforms robots from tools that require specialized training to assistants that anyone can direct.

In this chapter, we'll build a complete voice-to-action pipeline: capturing audio from a microphone, transcribing it with Whisper, parsing the transcription into commands, and publishing robot movements via ROS 2. By the end, you'll have a working voice-controlled robot that responds to spoken commands like "move forward" or "turn left."

## Foundational Concepts

### Whisper Architecture

**Simple Explanation**: Whisper is a neural network that converts speech audio into text. You speak into a microphone, Whisper processes the audio waveform, and outputs what you said as written text. It's trained on 680,000 hours of multilingual audio, so it understands many languages and accents without additional training.

**Detailed Explanation**: Whisper uses a transformer encoder-decoder architecture. The **encoder** processes 30-second audio chunks (as mel spectrograms—visual representations of sound frequencies over time) and produces a sequence of audio features. The **decoder** then generates text tokens autoregressively, using both the audio features and previously generated tokens.

Key architectural features:
- **Multitask Training**: Trained simultaneously on transcription, translation, language identification, and voice activity detection
- **Sequence-to-Sequence**: Maps variable-length audio to variable-length text
- **Multilingual**: Supports 99 languages with automatic language detection
- **Robust to Noise**: Trained on diverse audio quality, handles background noise well

Model sizes range from 39M parameters (tiny) to 1.5B parameters (large), trading off speed for accuracy:

| Model | Parameters | English WER | Speed (RTF) |
|-------|------------|-------------|-------------|
| tiny | 39M | 9.0% | 0.02 |
| base | 74M | 6.7% | 0.03 |
| small | 244M | 4.3% | 0.08 |
| medium | 769M | 3.2% | 0.17 |
| large | 1.5B | 2.7% | 0.35 |

*WER = Word Error Rate (lower is better), RTF = Real-Time Factor (lower is faster)*

For robotics, the **base** or **small** model provides the best balance between accuracy and latency.

### Audio Processing Pipeline

**Simple Explanation**: Before Whisper can transcribe speech, we need to capture audio correctly. This involves selecting the right microphone, recording audio at the proper sample rate (16 kHz), and optionally detecting when speech starts and stops.

**Detailed Explanation**: The audio processing pipeline for voice-controlled robotics consists of several stages:

```
Microphone → Audio Capture → Preprocessing → Whisper → Post-processing → Command
     │              │              │              │              │
   Hardware    Sample Rate    Noise Filter    Transcribe    Parse Text
              (16000 Hz)     Normalization                 to Commands
```

**1. Audio Capture**: Using libraries like `sounddevice`, we capture audio from the system microphone. Key parameters:
- **Sample Rate**: 16000 Hz (Whisper's native rate)
- **Channels**: 1 (mono)
- **Format**: float32 (normalized -1.0 to 1.0)
- **Chunk Duration**: 2-5 seconds (balance between latency and context)

**2. Voice Activity Detection (VAD)**: Optional preprocessing to detect speech vs. silence:
```python
# Simple energy-based VAD
def is_speech(audio_chunk, threshold=0.01):
    rms = np.sqrt(np.mean(audio_chunk ** 2))
    return rms > threshold
```

**3. Preprocessing**: Normalize audio levels and optionally apply noise reduction:
```python
# Normalize to [-1, 1] range
audio = audio / np.max(np.abs(audio))
```

**4. Whisper Inference**: Pass preprocessed audio to Whisper:
```python
result = whisper_model.transcribe(audio, language='en')
text = result['text']
confidence = 1.0 - result['segments'][0]['no_speech_prob']
```

### ROS 2 Integration Patterns

**Simple Explanation**: The voice node publishes two things: the raw transcribed text (for logging or display) and velocity commands (for robot movement). Other ROS 2 nodes can subscribe to either topic as needed.

**Detailed Explanation**: Integrating voice recognition with ROS 2 follows the publisher-subscriber pattern from [Module 1](../module-01-robotic-nervous-system/02-nodes-and-topics.md):

```
┌─────────────────────────────────────────────────────────────────┐
│                    WhisperVoiceNode                             │
│  ┌──────────────┐   ┌────────────┐   ┌──────────────────────┐  │
│  │ Audio Capture│ → │  Whisper   │ → │  Command Parser      │  │
│  │ (sounddevice)│   │ Transcribe │   │  (text → velocity)   │  │
│  └──────────────┘   └────────────┘   └──────────────────────┘  │
│           │                │                    │               │
└───────────┼────────────────┼────────────────────┼───────────────┘
            │                │                    │
            │       ┌────────▼────────┐   ┌───────▼───────┐
            │       │ /voice_command  │   │   /cmd_vel    │
            │       │   (String)      │   │   (Twist)     │
            │       └─────────────────┘   └───────────────┘
            │                                     │
   (audio stream)                         (motor control)
```

**Topic Design**:
- `/voice_command` (std_msgs/String): Raw transcription for logging, display, or further NLP processing
- `/cmd_vel` (geometry_msgs/Twist): Standard velocity topic consumed by robot controllers

**Node Lifecycle**:
1. Initialize Whisper model (once, at startup)
2. Start audio capture stream
3. Process audio chunks in continuous loop
4. Transcribe when speech detected
5. Parse and publish commands

This separation allows flexible architectures—you could replace the command parser with an LLM (from Chapter 1) for more complex reasoning, or route `/voice_command` to a conversation system.

## Practical Example: Voice-Controlled Robot

Let's examine a complete working example that captures voice commands and controls a robot.

### Whisper Voice Node

```python
#!/usr/bin/env python3
"""Whisper Voice Node - Capture audio, transcribe, and publish robot commands."""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import whisper
import sounddevice as sd

class WhisperVoiceNode(Node):
    """ROS 2 node that captures audio and transcribes using Whisper."""

    def __init__(self):
        super().__init__('whisper_voice_node')

        # Step 1: Load Whisper model (base is good balance of speed/accuracy)
        self.get_logger().info("Loading Whisper model: base")
        self.whisper_model = whisper.load_model("base")
        self.get_logger().info("Whisper model loaded successfully")

        # Step 2: Configure audio capture
        self.sample_rate = 16000  # Whisper expects 16kHz
        self.chunk_duration = 3.0  # 3 seconds per recording

        # Step 3: Create ROS 2 publishers
        self.voice_pub = self.create_publisher(String, '/voice_command', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Step 4: Define command mapping
        self.command_map = {
            'forward': {'linear_x': 0.5, 'angular_z': 0.0},
            'move forward': {'linear_x': 0.5, 'angular_z': 0.0},
            'backward': {'linear_x': -0.5, 'angular_z': 0.0},
            'turn left': {'linear_x': 0.0, 'angular_z': 0.5},
            'turn right': {'linear_x': 0.0, 'angular_z': -0.5},
            'stop': {'linear_x': 0.0, 'angular_z': 0.0},
        }

    def capture_audio(self):
        """Capture audio chunk from microphone."""
        samples = int(self.sample_rate * self.chunk_duration)
        self.get_logger().info(f"Recording {self.chunk_duration}s...")

        # Record audio using sounddevice
        recording = sd.rec(samples, samplerate=self.sample_rate,
                          channels=1, dtype='float32')
        sd.wait()  # Wait for recording to complete

        return recording.flatten()

    def transcribe(self, audio):
        """Transcribe audio using Whisper."""
        result = self.whisper_model.transcribe(
            audio,
            language='en',
            fp16=False  # Use FP32 for CPU compatibility
        )
        text = result['text'].strip().lower()

        # Calculate confidence from no_speech_prob
        segments = result.get('segments', [])
        if segments:
            confidence = 1.0 - segments[0].get('no_speech_prob', 0)
        else:
            confidence = 0.5

        return text, confidence

    def parse_command(self, text):
        """Parse transcription into velocity command."""
        # Try exact match first
        if text in self.command_map:
            return self.command_map[text]

        # Try partial match
        for keyword, command in self.command_map.items():
            if keyword in text:
                return command

        return None  # Unrecognized

    def process_voice_command(self):
        """Complete pipeline: capture → transcribe → parse → publish."""
        # Step 1: Capture audio
        audio = self.capture_audio()

        # Step 2: Transcribe with Whisper
        text, confidence = self.transcribe(audio)
        self.get_logger().info(f"Transcribed: '{text}' (confidence: {confidence:.2f})")

        # Step 3: Publish raw transcription
        voice_msg = String()
        voice_msg.data = text
        self.voice_pub.publish(voice_msg)

        # Step 4: Check confidence threshold
        if confidence < 0.7:
            self.get_logger().warn("Low confidence, ignoring")
            return

        # Step 5: Parse and publish velocity command
        command = self.parse_command(text)
        if command:
            twist = Twist()
            twist.linear.x = command['linear_x']
            twist.angular.z = command['angular_z']
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Published: linear={twist.linear.x}, angular={twist.angular.z}")
        else:
            self.get_logger().warn(f"Unrecognized command: '{text}'")
```

**Key Components Explained**:

**Whisper Model Loading (Step 1)**: We load the model once at startup to avoid repeated initialization overhead. The `base` model provides good accuracy (~6.7% WER) with fast inference (~30ms per second of audio on CPU).

**Audio Configuration (Step 2)**: Whisper expects 16 kHz mono audio. The 3-second chunk duration provides enough context for complete commands while keeping latency acceptable.

**Dual Publishers (Step 3)**: Publishing both raw text (`/voice_command`) and parsed commands (`/cmd_vel`) enables flexible system design—you might want to display transcriptions on a screen while simultaneously controlling the robot.

**Command Mapping (Step 4)**: A dictionary maps recognized phrases to velocity parameters. This is extensible—add new commands by adding entries to the dictionary.

### Audio Capture Module

```python
"""Audio capture utilities with Voice Activity Detection."""

import numpy as np
import sounddevice as sd

class AudioCapture:
    """Audio capture with device selection and VAD."""

    def __init__(self, device_index=None, sample_rate=16000):
        self.device_index = device_index
        self.sample_rate = sample_rate

    @staticmethod
    def list_devices():
        """List available audio input devices."""
        devices = sd.query_devices()
        print("Available Input Devices:")
        for i, d in enumerate(devices):
            if d['max_input_channels'] > 0:
                print(f"  [{i}] {d['name']}")

    def record(self, duration):
        """Record audio for specified duration."""
        samples = int(self.sample_rate * duration)
        recording = sd.rec(samples, samplerate=self.sample_rate,
                          channels=1, dtype='float32',
                          device=self.device_index)
        sd.wait()
        return recording.flatten()

    def record_with_vad(self, max_duration=10.0, silence_threshold=0.01):
        """Record until silence is detected after speech."""
        chunk_size = int(self.sample_rate * 0.1)  # 100ms chunks
        audio_buffer = []
        speech_detected = False
        silence_chunks = 0

        with sd.InputStream(samplerate=self.sample_rate, channels=1,
                           dtype='float32', device=self.device_index) as stream:
            while len(audio_buffer) * chunk_size < max_duration * self.sample_rate:
                chunk, _ = stream.read(chunk_size)
                audio_buffer.append(chunk.flatten())

                # Simple energy-based VAD
                rms = np.sqrt(np.mean(chunk ** 2))
                if rms > silence_threshold:
                    speech_detected = True
                    silence_chunks = 0
                else:
                    silence_chunks += 1

                # Stop after 1 second of silence following speech
                if speech_detected and silence_chunks > 10:
                    break

        return np.concatenate(audio_buffer)
```

**Voice Activity Detection (VAD)**: The `record_with_vad` method uses energy-based detection to determine when speech starts and stops. This reduces unnecessary Whisper calls during silence and provides natural pauses between commands.

### Command Publisher Module

```python
"""Natural language command parsing with speed modifiers."""

import re
from geometry_msgs.msg import Twist

class CommandPublisher:
    """Parse voice commands with speed modifiers."""

    def __init__(self):
        # Velocity configurations
        self.velocities = {
            'slow': {'linear': 0.2, 'angular': 0.3},
            'normal': {'linear': 0.5, 'angular': 0.5},
            'fast': {'linear': 0.8, 'angular': 0.8}
        }

        # Direction patterns
        self.patterns = {
            'forward': re.compile(r'(move|go)?\s*(forward|ahead)', re.I),
            'backward': re.compile(r'(move|go)?\s*(backward|back)', re.I),
            'left': re.compile(r'(turn|rotate)?\s*left', re.I),
            'right': re.compile(r'(turn|rotate)?\s*right', re.I),
            'stop': re.compile(r'(stop|halt)', re.I),
        }

    def parse(self, text):
        """Parse command into Twist message."""
        text = text.lower()

        # Detect speed modifier
        speed = 'normal'
        if 'slow' in text:
            speed = 'slow'
        elif 'fast' in text:
            speed = 'fast'

        vel = self.velocities[speed]

        # Detect direction
        if self.patterns['forward'].search(text):
            return self._twist(vel['linear'], 0.0)
        elif self.patterns['backward'].search(text):
            return self._twist(-vel['linear'], 0.0)
        elif self.patterns['left'].search(text):
            return self._twist(0.0, vel['angular'])
        elif self.patterns['right'].search(text):
            return self._twist(0.0, -vel['angular'])
        elif self.patterns['stop'].search(text):
            return self._twist(0.0, 0.0)

        return None

    def _twist(self, linear_x, angular_z):
        """Create Twist message with safety bounds."""
        twist = Twist()
        twist.linear.x = max(-1.0, min(1.0, linear_x))
        twist.angular.z = max(-1.0, min(1.0, angular_z))
        return twist
```

**Speed Modifiers**: This pattern enables natural variations like "move forward slowly" or "turn right fast" by combining direction detection with speed modifier parsing.

### Running the Example

The complete code is available in `solutions/chapter-02/`. To run it:

```bash
# 1. Install dependencies
pip install -r requirements.txt

# 2. Source ROS 2
source /opt/ros/humble/setup.bash

# 3. Run the voice node
python3 whisper_voice_node.py
```

**Expected Behavior**:
```
Loading Whisper model: base
Whisper model loaded successfully
Starting continuous voice recognition...
Speak commands: 'forward', 'backward', 'left', 'right', 'stop'

Recording 3.0s...
Transcribed: 'move forward' (confidence: 0.94)
Published: linear=0.5, angular=0.0

Recording 3.0s...
Transcribed: 'turn left slowly' (confidence: 0.91)
Published: linear=0.0, angular=0.3
```

## Integration with ROS 2

This example builds on ROS 2 concepts from [Module 1](../module-01-robotic-nervous-system/02-nodes-and-topics.md):

**Publishers and Topics**: We use two publishers:
- `/voice_command` (String): For debugging, logging, or connecting to other NLP systems
- `/cmd_vel` (Twist): Standard velocity topic that robot controllers subscribe to

**Message Types**: Both `std_msgs/String` and `geometry_msgs/Twist` are standard ROS 2 message types, ensuring compatibility with existing robot stacks.

**Testing in Simulation** (from [Module 2](../module-02-robot-perception/02-getting-started-with-gazebo.md)): Before deploying voice control to physical hardware, test in Gazebo simulation to validate the integration safely. The simulation environment allows you to verify transcription accuracy and robot response without risk of hardware damage.

**Node Parameters**: ROS 2 parameters (declared with `declare_parameter`) allow runtime configuration:
```python
self.declare_parameter('model_size', 'base')
model = self.get_parameter('model_size').value
```

**Continuous Operation**: The node runs in a continuous loop, processing voice commands as they occur, demonstrating the event-driven nature of ROS 2.

## Common Pitfalls and Troubleshooting

### Issue 1: Low Transcription Accuracy

**Symptom**: Commands frequently misheard or not recognized

**Causes**:
- Background noise
- Microphone quality
- Speaking too fast or too quietly
- Wrong language setting

**Solutions**:
```python
# Use a larger model for better accuracy
self.whisper_model = whisper.load_model("small")  # or "medium"

# Add noise words to filter
noise_words = ['um', 'uh', 'like', 'you know']
text = ' '.join(w for w in text.split() if w not in noise_words)

# Lower confidence threshold for testing
if confidence < 0.5:  # Instead of 0.7
    self.get_logger().warn("Low confidence")
```

**Microphone Tips**:
- Position microphone 6-12 inches from mouth
- Use a directional microphone in noisy environments
- Test with `python3 audio_capture.py` before deploying

### Issue 2: High Latency

**Symptom**: Noticeable delay (>1 second) between speaking and robot movement

**Causes**:
- Large Whisper model
- CPU-only inference
- Long audio chunks

**Solutions**:
```python
# Use smaller model
self.whisper_model = whisper.load_model("tiny")  # Fastest

# Shorter chunk duration
self.chunk_duration = 2.0  # Instead of 3.0

# Enable GPU if available
result = self.whisper_model.transcribe(audio, fp16=True)  # Requires CUDA
```

**Latency Breakdown**:
| Component | Typical Latency |
|-----------|-----------------|
| Audio capture | 2-3s (chunk duration) |
| Whisper (base, CPU) | 0.5-1.0s |
| Command parsing | ~1ms |
| ROS 2 publish | ~1ms |
| **Total** | **2.5-4.0s** |

For real-time applications, consider streaming ASR alternatives or shorter chunks with VAD.

### Issue 3: Microphone Not Detected

**Symptom**: "No input devices found" or no audio recorded

**Solutions**:
```bash
# Linux: Check ALSA devices
arecord -l

# Check permissions
sudo usermod -a -G audio $USER
# Then logout/login

# Python: List devices
python3 -c "import sounddevice; print(sounddevice.query_devices())"

# Specify device explicitly
capture = AudioCapture(device_index=1)  # Use device 1
```

### Issue 4: Commands Not Recognized

**Symptom**: Whisper transcribes correctly but robot doesn't respond

**Causes**:
- Transcription doesn't match command patterns
- Extra words in transcription

**Solutions**:
```python
# Add more variations to command map
self.command_map = {
    'forward': {...},
    'go forward': {...},
    'move forward': {...},
    'move ahead': {...},
    'straight': {...},
    # ...
}

# Use fuzzy matching
from difflib import get_close_matches
keywords = list(self.command_map.keys())
matches = get_close_matches(text, keywords, n=1, cutoff=0.6)
if matches:
    return self.command_map[matches[0]]
```

## Chapter Summary

**Key Takeaways**:

1. **Whisper for Robotics**: OpenAI's Whisper provides robust, offline speech recognition ideal for robot control—no internet required, handles noise well, and supports multiple languages

2. **Audio Pipeline**: Capture → Preprocess → Transcribe → Parse → Publish forms the complete voice-to-action pipeline; each stage is independently testable

3. **Model Selection**: The `base` model (74M parameters) provides the best balance of accuracy (~6.7% WER) and speed (~30ms/s audio) for robotics applications

4. **Voice Activity Detection**: Using energy-based VAD reduces unnecessary transcription attempts and provides natural pauses between commands

5. **Command Mapping**: A dictionary-based approach with regex patterns handles natural language variations ("move forward", "go ahead", "forward")

6. **Dual Publishing**: Publish both raw transcription and parsed commands to enable flexible system design and debugging

**Critical Code Patterns**:

- **Whisper transcription**: `result = model.transcribe(audio, language='en', fp16=False)`
- **Confidence estimation**: `confidence = 1.0 - segment['no_speech_prob']`
- **Audio capture**: `sd.rec(samples, samplerate=16000, channels=1, dtype='float32')`
- **Safety bounds**: Always clamp velocities to safe ranges before publishing

**Common Pitfalls**:

- **Low accuracy**: Use larger models or improve microphone positioning
- **High latency**: Use smaller models, shorter chunks, or GPU acceleration
- **Microphone issues**: Check permissions, list devices, specify device index
- **Command recognition**: Add more pattern variations, use fuzzy matching

**Prerequisites for Next Chapter**:

- Understanding of audio capture and Whisper transcription
- Familiarity with ROS 2 publishers and String/Twist message types
- Experience running and modifying voice-controlled nodes
- Awareness of latency trade-offs in real-time systems

**Additional Resources**:

- [OpenAI Whisper GitHub Repository](https://github.com/openai/whisper)
- [Whisper Paper: Robust Speech Recognition via Large-Scale Weak Supervision](https://arxiv.org/abs/2212.04356)
- [sounddevice Documentation](https://python-sounddevice.readthedocs.io/)
- [ROS 2 rclpy Publisher/Subscriber Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

---

**Next Chapter**: [Cognitive Planning with LLMs](03-cognitive-planning-llms.md) - Learn how to decompose complex commands like "clean the room" into executable action sequences using LLM-powered task planning.
