#!/usr/bin/env python3
"""
Audio Capture Utilities for Voice-Controlled Robotics

This module provides audio device enumeration, selection, and capture
functionality for integrating voice recognition with ROS 2 robots.

Features:
- Device enumeration and selection
- Multiple audio format support
- Voice activity detection (VAD)
- Buffer management for continuous recording

Usage:
    from audio_capture import AudioCapture

    # List available devices
    AudioCapture.list_devices()

    # Capture audio
    capture = AudioCapture(device_index=0)
    audio_data = capture.record(duration=3.0)
"""

import numpy as np
from typing import Optional, List, Dict

# Sounddevice import with fallback
try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
except ImportError:
    SOUNDDEVICE_AVAILABLE = False


class AudioCapture:
    """Audio capture class with device management and recording utilities."""

    def __init__(
        self,
        device_index: Optional[int] = None,
        sample_rate: int = 16000,
        channels: int = 1,
        dtype: str = 'float32'
    ):
        """Initialize audio capture.

        Args:
            device_index: Audio input device index (None for default)
            sample_rate: Sample rate in Hz (16000 for Whisper)
            channels: Number of audio channels (1 for mono)
            dtype: Audio data type
        """
        self.device_index = device_index
        self.sample_rate = sample_rate
        self.channels = channels
        self.dtype = dtype

        if not SOUNDDEVICE_AVAILABLE:
            raise ImportError(
                "sounddevice is not installed. "
                "Install with: pip install sounddevice"
            )

        # Validate device
        if device_index is not None:
            devices = sd.query_devices()
            if device_index >= len(devices):
                raise ValueError(f"Device index {device_index} not found")

    @staticmethod
    def list_devices() -> List[Dict]:
        """List all available audio input devices.

        Returns:
            List of device information dictionaries
        """
        if not SOUNDDEVICE_AVAILABLE:
            print("sounddevice not available")
            return []

        devices = sd.query_devices()
        input_devices = []

        print("\n" + "=" * 60)
        print("Available Audio Input Devices")
        print("=" * 60)

        for i, device in enumerate(devices):
            if device['max_input_channels'] > 0:
                input_devices.append({
                    'index': i,
                    'name': device['name'],
                    'channels': device['max_input_channels'],
                    'sample_rate': device['default_samplerate']
                })
                print(f"[{i}] {device['name']}")
                print(f"    Channels: {device['max_input_channels']}, "
                      f"Sample Rate: {device['default_samplerate']} Hz")

        if not input_devices:
            print("No input devices found!")

        print("=" * 60 + "\n")
        return input_devices

    @staticmethod
    def get_default_device() -> Optional[int]:
        """Get the default input device index.

        Returns:
            Device index or None if no default
        """
        if not SOUNDDEVICE_AVAILABLE:
            return None

        try:
            return sd.default.device[0]  # Input device index
        except Exception:
            return None

    def record(self, duration: float) -> Optional[np.ndarray]:
        """Record audio for specified duration.

        Args:
            duration: Recording duration in seconds

        Returns:
            Audio data as numpy array, or None on failure
        """
        samples = int(self.sample_rate * duration)

        try:
            print(f"Recording {duration}s of audio...")
            recording = sd.rec(
                samples,
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype=self.dtype,
                device=self.device_index
            )
            sd.wait()  # Wait for recording to complete
            print("Recording complete")

            # Flatten to 1D array for Whisper
            return recording.flatten()

        except Exception as e:
            print(f"Recording error: {e}")
            return None

    def record_with_vad(
        self,
        max_duration: float = 10.0,
        silence_threshold: float = 0.01,
        silence_duration: float = 1.0
    ) -> Optional[np.ndarray]:
        """Record audio with voice activity detection (VAD).

        Stops recording after detecting silence following speech.

        Args:
            max_duration: Maximum recording duration in seconds
            silence_threshold: RMS threshold for silence detection
            silence_duration: Required silence duration to stop (seconds)

        Returns:
            Audio data as numpy array, or None on failure
        """
        samples = int(self.sample_rate * max_duration)
        chunk_size = int(self.sample_rate * 0.1)  # 100ms chunks
        silence_chunks = int(silence_duration / 0.1)

        audio_buffer = []
        speech_detected = False
        consecutive_silence = 0

        print(f"Recording with VAD (max {max_duration}s)...")
        print("Speak now, recording will stop after silence...")

        try:
            with sd.InputStream(
                samplerate=self.sample_rate,
                channels=self.channels,
                dtype=self.dtype,
                device=self.device_index
            ) as stream:
                while len(audio_buffer) * chunk_size < samples:
                    chunk, _ = stream.read(chunk_size)
                    audio_buffer.append(chunk.flatten())

                    # Calculate RMS for VAD
                    rms = np.sqrt(np.mean(chunk ** 2))

                    if rms > silence_threshold:
                        speech_detected = True
                        consecutive_silence = 0
                    else:
                        consecutive_silence += 1

                    # Stop if speech was detected and now silent
                    if speech_detected and consecutive_silence >= silence_chunks:
                        print("Silence detected, stopping recording")
                        break

            audio_data = np.concatenate(audio_buffer)
            print(f"Recorded {len(audio_data) / self.sample_rate:.2f}s of audio")
            return audio_data

        except Exception as e:
            print(f"VAD recording error: {e}")
            return None

    def test_microphone(self, duration: float = 2.0) -> bool:
        """Test microphone by recording and checking audio levels.

        Args:
            duration: Test duration in seconds

        Returns:
            True if microphone is working, False otherwise
        """
        print(f"Testing microphone for {duration}s...")
        print("Please make some noise (speak or clap)...")

        audio = self.record(duration)
        if audio is None:
            print("FAIL: Could not record audio")
            return False

        # Check audio levels
        max_amplitude = np.max(np.abs(audio))
        rms = np.sqrt(np.mean(audio ** 2))

        print(f"Max amplitude: {max_amplitude:.4f}")
        print(f"RMS level: {rms:.4f}")

        if max_amplitude < 0.001:
            print("FAIL: Audio level too low - check microphone")
            return False

        print("PASS: Microphone is working")
        return True


def main():
    """Main function demonstrating audio capture utilities."""
    print("\n" + "=" * 60)
    print("Audio Capture Utilities Demo")
    print("=" * 60 + "\n")

    if not SOUNDDEVICE_AVAILABLE:
        print("ERROR: sounddevice not installed")
        print("Install with: pip install sounddevice")
        return

    # List available devices
    devices = AudioCapture.list_devices()

    if not devices:
        print("No input devices available. Exiting.")
        return

    # Use default device
    try:
        capture = AudioCapture()

        # Test microphone
        print("\n--- Microphone Test ---")
        if capture.test_microphone():
            # Record sample audio
            print("\n--- Sample Recording ---")
            audio = capture.record(duration=3.0)
            if audio is not None:
                print(f"Recorded {len(audio)} samples")
                print(f"Duration: {len(audio) / capture.sample_rate:.2f}s")
                print(f"Min: {np.min(audio):.4f}, Max: {np.max(audio):.4f}")

    except Exception as e:
        print(f"Error: {e}")

    print("\nDemo complete.")


if __name__ == "__main__":
    main()
