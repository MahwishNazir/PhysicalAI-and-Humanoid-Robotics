#!/usr/bin/env python3
"""
Whisper Voice Node for ROS 2

This example demonstrates how to integrate OpenAI's Whisper speech recognition
with ROS 2 for voice-controlled robot commands. The node captures audio from
a microphone, transcribes it using Whisper, and publishes recognized commands.

Prerequisites:
- ROS 2 Humble installed and sourced
- Python 3.8+
- Whisper installed (pip install openai-whisper)
- sounddevice installed (pip install sounddevice)
- Microphone access

Usage:
    python3 whisper_voice_node.py
"""

import os
import queue
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Whisper import with fallback
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("Warning: Whisper not installed. Install with: pip install openai-whisper")

# Sounddevice import with fallback
try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
except ImportError:
    SOUNDDEVICE_AVAILABLE = False
    print("Warning: sounddevice not installed. Install with: pip install sounddevice")


class WhisperVoiceNode(Node):
    """ROS 2 node that captures audio and transcribes using Whisper."""

    def __init__(self):
        super().__init__('whisper_voice_node')

        # Configuration parameters
        self.declare_parameter('model_size', 'base')  # tiny, base, small, medium, large
        self.declare_parameter('sample_rate', 16000)  # Whisper expects 16kHz
        self.declare_parameter('chunk_duration', 3.0)  # seconds per audio chunk
        self.declare_parameter('confidence_threshold', 0.7)  # minimum confidence
        self.declare_parameter('language', 'en')  # language hint for Whisper

        # Get parameters
        self.model_size = self.get_parameter('model_size').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.language = self.get_parameter('language').value

        # Initialize Whisper model
        if WHISPER_AVAILABLE:
            self.get_logger().info(f"Loading Whisper model: {self.model_size}")
            self.whisper_model = whisper.load_model(self.model_size)
            self.get_logger().info("Whisper model loaded successfully")
        else:
            self.whisper_model = None
            self.get_logger().warn("Whisper not available - running in demo mode")

        # Audio queue for thread-safe audio capture
        self.audio_queue = queue.Queue()

        # Create ROS 2 publishers
        self.voice_command_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info("Voice command publisher created on /voice_command")
        self.get_logger().info("Velocity publisher created on /cmd_vel")

        # Audio capture state
        self.is_recording = False
        self.audio_buffer = []

        # Command mapping for simple voice-to-action
        self.command_map = {
            'forward': {'linear_x': 0.5, 'angular_z': 0.0},
            'move forward': {'linear_x': 0.5, 'angular_z': 0.0},
            'go forward': {'linear_x': 0.5, 'angular_z': 0.0},
            'backward': {'linear_x': -0.5, 'angular_z': 0.0},
            'move backward': {'linear_x': -0.5, 'angular_z': 0.0},
            'go back': {'linear_x': -0.5, 'angular_z': 0.0},
            'left': {'linear_x': 0.0, 'angular_z': 0.5},
            'turn left': {'linear_x': 0.0, 'angular_z': 0.5},
            'right': {'linear_x': 0.0, 'angular_z': -0.5},
            'turn right': {'linear_x': 0.0, 'angular_z': -0.5},
            'stop': {'linear_x': 0.0, 'angular_z': 0.0},
            'halt': {'linear_x': 0.0, 'angular_z': 0.0},
        }

    def audio_callback(self, indata, frames, time_info, status):
        """Callback function for audio capture stream."""
        if status:
            self.get_logger().warn(f"Audio capture status: {status}")
        # Add audio data to queue
        self.audio_queue.put(indata.copy())

    def capture_audio_chunk(self):
        """Capture a chunk of audio from the microphone."""
        if not SOUNDDEVICE_AVAILABLE:
            self.get_logger().warn("sounddevice not available for audio capture")
            return None

        samples_needed = int(self.sample_rate * self.chunk_duration)
        audio_data = []

        self.get_logger().info(f"Recording {self.chunk_duration}s audio chunk...")

        try:
            # Record audio chunk
            recording = sd.rec(
                samples_needed,
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()  # Wait for recording to complete

            # Convert to the format Whisper expects
            audio_array = recording.flatten()

            self.get_logger().info(f"Captured {len(audio_array)} audio samples")
            return audio_array

        except Exception as e:
            self.get_logger().error(f"Audio capture error: {e}")
            return None

    def transcribe_audio(self, audio_data):
        """Transcribe audio data using Whisper.

        Args:
            audio_data: numpy array of audio samples (float32, 16kHz)

        Returns:
            dict with 'text' and 'confidence' keys, or None on failure
        """
        if not WHISPER_AVAILABLE or self.whisper_model is None:
            self.get_logger().warn("Whisper not available for transcription")
            return None

        if audio_data is None or len(audio_data) == 0:
            return None

        try:
            # Transcribe with Whisper
            result = self.whisper_model.transcribe(
                audio_data,
                language=self.language,
                fp16=False  # Use FP32 for CPU compatibility
            )

            text = result.get('text', '').strip().lower()

            # Calculate approximate confidence from segments
            segments = result.get('segments', [])
            if segments:
                avg_no_speech_prob = sum(
                    seg.get('no_speech_prob', 0) for seg in segments
                ) / len(segments)
                confidence = 1.0 - avg_no_speech_prob
            else:
                confidence = 0.5  # Default if no segment info

            self.get_logger().info(f"Transcribed: '{text}' (confidence: {confidence:.2f})")

            return {
                'text': text,
                'confidence': confidence
            }

        except Exception as e:
            self.get_logger().error(f"Transcription error: {e}")
            return None

    def parse_command(self, transcription):
        """Parse transcription into robot command.

        Args:
            transcription: dict with 'text' and 'confidence'

        Returns:
            dict with 'linear_x' and 'angular_z', or None if not recognized
        """
        if transcription is None:
            return None

        text = transcription['text']
        confidence = transcription['confidence']

        # Check confidence threshold
        if confidence < self.confidence_threshold:
            self.get_logger().warn(
                f"Low confidence ({confidence:.2f} < {self.confidence_threshold}), "
                "ignoring command"
            )
            return None

        # Try exact match first
        if text in self.command_map:
            return self.command_map[text]

        # Try partial match
        for keyword, command in self.command_map.items():
            if keyword in text:
                self.get_logger().info(f"Matched keyword '{keyword}' in '{text}'")
                return command

        self.get_logger().warn(f"Unrecognized command: '{text}'")
        return None

    def publish_command(self, command, original_text):
        """Publish command to ROS 2 topics.

        Args:
            command: dict with 'linear_x' and 'angular_z'
            original_text: original transcribed text
        """
        # Publish transcribed text to /voice_command
        text_msg = String()
        text_msg.data = original_text
        self.voice_command_pub.publish(text_msg)

        # Publish velocity command to /cmd_vel
        twist_msg = Twist()
        twist_msg.linear.x = float(command.get('linear_x', 0.0))
        twist_msg.angular.z = float(command.get('angular_z', 0.0))

        # Safety bounds
        twist_msg.linear.x = max(-1.0, min(1.0, twist_msg.linear.x))
        twist_msg.angular.z = max(-1.0, min(1.0, twist_msg.angular.z))

        self.cmd_vel_pub.publish(twist_msg)

        self.get_logger().info(
            f"Published: '{original_text}' -> "
            f"linear={twist_msg.linear.x}, angular={twist_msg.angular.z}"
        )

    def process_voice_command(self):
        """Complete pipeline: capture -> transcribe -> parse -> publish."""
        # Step 1: Capture audio
        audio_data = self.capture_audio_chunk()
        if audio_data is None:
            return False

        # Step 2: Transcribe with Whisper
        transcription = self.transcribe_audio(audio_data)
        if transcription is None:
            return False

        # Step 3: Parse into command
        command = self.parse_command(transcription)
        if command is None:
            return False

        # Step 4: Publish to ROS 2
        self.publish_command(command, transcription['text'])
        return True

    def run_continuous(self):
        """Run continuous voice recognition loop."""
        self.get_logger().info("Starting continuous voice recognition...")
        self.get_logger().info("Speak commands: 'forward', 'backward', 'left', 'right', 'stop'")
        self.get_logger().info("Press Ctrl+C to exit")

        while rclpy.ok():
            try:
                success = self.process_voice_command()
                if not success:
                    self.get_logger().debug("No valid command detected")
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f"Error in voice processing: {e}")


def main(args=None):
    """Main function to run the Whisper voice node."""
    rclpy.init(args=args)

    print("\n" + "=" * 60)
    print("Whisper Voice Control Node")
    print("=" * 60)

    if not WHISPER_AVAILABLE:
        print("\nERROR: Whisper is not installed!")
        print("Install with: pip install openai-whisper")
        print("\nRunning in demo mode with simulated commands...\n")

    if not SOUNDDEVICE_AVAILABLE:
        print("\nERROR: sounddevice is not installed!")
        print("Install with: pip install sounddevice")
        print("\nRunning in demo mode with simulated audio...\n")

    try:
        node = WhisperVoiceNode()

        if WHISPER_AVAILABLE and SOUNDDEVICE_AVAILABLE:
            # Run continuous voice recognition
            node.run_continuous()
        else:
            # Demo mode: simulate some commands
            print("Demo mode: Simulating voice commands...")
            demo_commands = [
                "move forward",
                "turn left",
                "stop"
            ]
            import time
            for cmd in demo_commands:
                print(f"\n[Demo] Simulating command: '{cmd}'")
                # Simulate transcription result
                transcription = {'text': cmd, 'confidence': 0.95}
                command = node.parse_command(transcription)
                if command:
                    node.publish_command(command, cmd)
                time.sleep(1)

            print("\nDemo complete. Install dependencies for real voice control.")
            rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nShutdown requested by user.")
    except Exception as e:
        print(f"\nFatal error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()
        print("Node shutdown complete.")


if __name__ == "__main__":
    main()
