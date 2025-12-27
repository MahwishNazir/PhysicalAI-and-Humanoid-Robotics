#!/usr/bin/env python3
"""
Voice Interface Module for Autonomous Humanoid

Integrates Whisper speech recognition with the main orchestrator,
providing voice command capture functionality.
"""

import numpy as np
from typing import Optional

# Optional imports with fallbacks
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False

try:
    import sounddevice as sd
    SOUNDDEVICE_AVAILABLE = True
except ImportError:
    SOUNDDEVICE_AVAILABLE = False


class VoiceInterface:
    """Voice command interface using Whisper."""

    def __init__(self, parent_node, model_size: str = "base"):
        """Initialize voice interface.

        Args:
            parent_node: Parent ROS 2 node for logging
            model_size: Whisper model size (tiny, base, small, medium, large)
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.sample_rate = 16000
        self.chunk_duration = 3.0

        # Load Whisper model
        if WHISPER_AVAILABLE:
            self.logger.info(f"Loading Whisper model: {model_size}")
            self.model = whisper.load_model(model_size)
            self.logger.info("Whisper model loaded")
        else:
            self.model = None
            self.logger.warn("Whisper not available")

    def capture_command(self) -> Optional[str]:
        """Capture and transcribe a voice command.

        Returns:
            Transcribed command string, or None if capture fails
        """
        if not WHISPER_AVAILABLE or not SOUNDDEVICE_AVAILABLE:
            return None

        try:
            # Capture audio
            samples = int(self.sample_rate * self.chunk_duration)
            self.logger.info(f"Listening for {self.chunk_duration}s...")

            recording = sd.rec(
                samples,
                samplerate=self.sample_rate,
                channels=1,
                dtype='float32'
            )
            sd.wait()

            audio = recording.flatten()

            # Transcribe
            result = self.model.transcribe(
                audio,
                language='en',
                fp16=False
            )

            text = result.get('text', '').strip().lower()

            # Check confidence
            segments = result.get('segments', [])
            if segments:
                confidence = 1.0 - segments[0].get('no_speech_prob', 0)
            else:
                confidence = 0.5

            if confidence < 0.6:
                self.logger.warn(f"Low confidence ({confidence:.2f}), ignoring")
                return None

            self.logger.info(f"Transcribed: '{text}' (confidence: {confidence:.2f})")
            return text

        except Exception as e:
            self.logger.error(f"Voice capture error: {e}")
            return None

    def is_available(self) -> bool:
        """Check if voice interface is available."""
        return WHISPER_AVAILABLE and SOUNDDEVICE_AVAILABLE and self.model is not None
