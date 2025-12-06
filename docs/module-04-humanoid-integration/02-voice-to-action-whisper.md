---
title: "Voice-to-Action with OpenAI Whisper"
sidebar_label: "Whisper Integration"
sidebar_position: 2
description: "Real-time speech recognition for robot control using OpenAI Whisper"
tags: [whisper, speech-recognition, voice-control, asr]
keywords: [OpenAI Whisper, speech recognition, ASR, voice commands, ROS 2 audio]
difficulty: intermediate
estimated_time: "2 hours"
prerequisites: ["Chapter 1: Introduction to VLA"]
---

# Voice-to-Action with OpenAI Whisper

*Content coming soon. This chapter will cover:*

## Automatic Speech Recognition (ASR) Fundamentals

- History: From Hidden Markov Models to Deep Learning
- How ASR works: Audio → Features → Text
- Challenges: Accents, noise, context, vocabulary
- Real-time vs offline processing
- Evaluation metrics: Word Error Rate (WER)

## OpenAI Whisper Overview

### Why Whisper?

- State-of-the-art accuracy across languages
- Robust to accents and background noise
- Open-source and free to use
- Multiple model sizes (tiny → large)
- Supports 99+ languages

### Architecture

- Encoder-decoder transformer
- Trained on 680,000 hours of multilingual data
- Multitask training: transcription, translation, language detection
- Attention-based sequence-to-sequence

### Model Sizes and Trade-offs

| Model | Parameters | Speed | Accuracy | Use Case |
|-------|-----------|-------|----------|----------|
| tiny | 39M | ~32x | Good | Real-time, low latency |
| base | 74M | ~16x | Better | Balanced |
| small | 244M | ~6x | Great | Most applications |
| medium | 769M | ~2x | Excellent | High accuracy needed |
| large | 1550M | 1x | Best | Offline, research |

## Installation and Setup

### Installing Whisper

```bash
# Using pip
pip install openai-whisper

# Dependencies
pip install torch torchaudio
```

### Testing Whisper

```python
import whisper

model = whisper.load_model("base")
result = model.transcribe("audio.mp3")
print(result["text"])
```

### Model Selection

- Real-time robotics: **tiny** or **base**
- Accuracy-critical: **small** or **medium**
- Offline analysis: **large**

## Integrating Whisper with ROS 2

### Architecture

```
Microphone (audio_common)
    ↓
Audio Stream
    ↓
Whisper Node
    ↓
Transcription (String msg)
    ↓
Command Parser
    ↓
Robot Actions
```

### ROS 2 Audio Package

- `audio_common`: Audio capture and playback
- Publishing audio streams
- Subscribing to microphone input

### Custom Whisper Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import numpy as np

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.publisher = self.create_publisher(String, 'voice_command', 10)
        self.model = whisper.load_model("base")
        # Audio subscriber and processing...
```

## Real-Time Processing

### Challenges

- Whisper processes entire audio files
- Need continuous streaming
- Latency requirements (< 1 second ideal)
- Buffer management

### Solutions

**1. Chunk-based Processing**
- Split audio into fixed chunks (e.g., 5 seconds)
- Process each chunk independently
- Trade-off: May cut off words

**2. Voice Activity Detection (VAD)**
- Detect when speech starts and stops
- Only transcribe speech segments
- Reduces unnecessary processing

**3. Faster-Whisper**
- Optimized implementation using CTranslate2
- 4x faster than original Whisper
- Lower memory usage

```bash
pip install faster-whisper
```

## Voice Activity Detection (VAD)

### Why VAD?

- Don't process silence
- Detect natural command boundaries
- Reduce computational load
- Improve user experience

### Webrtcvad

```python
import webrtcvad

vad = webrtcvad.Vad(3)  # Aggressiveness 0-3
is_speech = vad.is_speech(audio_frame, sample_rate)
```

### Integration Pattern

```
Continuous Audio Stream
    ↓
VAD: Speech detected?
    ↓ (yes)
Buffer audio
    ↓
Silence detected
    ↓
Send buffer to Whisper
    ↓
Transcription
```

## Handling Multilingual Commands

### Language Detection

Whisper automatically detects language:

```python
result = model.transcribe("audio.mp3")
detected_language = result["language"]
```

### Forcing a Language

```python
result = model.transcribe("audio.mp3", language="en")
```

### Multi-Language Support

- Spanish: "Mueve el robot a la derecha"
- French: "Déplace le robot à droite"
- Chinese: "将机器人移到右边"

## Robustness to Noise

### Whisper's Built-in Robustness

- Trained on diverse audio conditions
- Handles background noise reasonably well
- Performs better than most ASR systems

### Improving Performance

**Audio Preprocessing**
- Noise reduction (noisereduce library)
- Automatic gain control
- High-pass filter for low-frequency noise

**Hardware Considerations**
- Use directional microphones
- Microphone arrays for beamforming
- Position microphone close to user

## Latency Optimization

### Strategies

1. **Use smaller models**: tiny or base for real-time
2. **GPU acceleration**: Run Whisper on GPU
3. **Faster-Whisper**: 4x speedup
4. **Streaming approaches**: Process while recording
5. **Prompt caching**: For repeated phrases

### Benchmarks

| Model | CPU (i7) | GPU (RTX 3080) |
|-------|---------|----------------|
| tiny | 0.5s | 0.1s |
| base | 1.0s | 0.2s |
| small | 3.0s | 0.5s |
| medium | 8.0s | 1.2s |

## Command Parsing

### From Text to Intent

```python
# Input: "Move forward 2 meters"
# Output: {"action": "move", "direction": "forward", "distance": 2}
```

### Approaches

**1. Rule-based Parsing**
- Regular expressions
- Keyword matching
- Simple and fast

**2. LLM-based Parsing**
- More flexible
- Handles variations
- Can clarify ambiguity

## Error Handling

### Common Issues

- Misrecognition: "Move" → "Groove"
- Ambiguous commands: "Go there" (where?)
- Out-of-vocabulary: Technical terms
- Partial transcriptions

### Solutions

**Confidence Scores**
```python
# Check if transcription is reliable
if result["confidence"] < 0.7:
    request_repeat()
```

**Clarification Dialogues**
```python
if command_is_ambiguous(text):
    robot_says("Did you mean X or Y?")
```

**Command Validation**
- Whitelist of allowed commands
- Safety checks before execution

## Hands-On Projects

### Project 1: Simple Voice Command
- Record audio
- Transcribe with Whisper
- Print the result
- Test with various accents and noise levels

### Project 2: ROS 2 Integration
- Create Whisper ROS 2 node
- Subscribe to audio topic
- Publish transcriptions
- Test in simulation

### Project 3: Real-Time Voice Control
- Implement VAD
- Use faster-whisper for low latency
- Control robot velocity with voice
- Add safety keywords ("stop", "emergency")

### Project 4: Multi-Language Support
- Detect language automatically
- Support 3+ languages
- Language-specific command sets

## Performance Tips

1. **Start with small models** for prototyping
2. **Use GPU** if available
3. **Implement VAD** to reduce processing
4. **Cache models** on node initialization
5. **Profile and optimize** bottlenecks

## Next Steps

In the next chapter, we'll connect Whisper transcriptions to **Large Language Models** for cognitive planning, transforming "Clean the room" into executable robot actions.

*For now, explore [OpenAI Whisper on GitHub](https://github.com/openai/whisper) and [faster-whisper](https://github.com/guillaumekln/faster-whisper).*
