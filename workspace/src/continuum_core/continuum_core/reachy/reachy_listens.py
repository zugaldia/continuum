import asyncio
from collections import deque
from typing import Callable
from typing import Optional, Deque

import numpy as np
import numpy.typing as npt
from pydantic import BaseModel
from rclpy.timer import Timer
from reachy_mini import ReachyMini

from continuum.audio import AudioManager
from continuum_core.reachy.models import ReachyAudio
from continuum_core.shared.base_node import BaseNode

# Audio recording configuration (poll every 10ms to capture samples)
# Reachy produces ~20ms audio chunks, so poll at least 2x that rate?
AUDIO_POLL_PERIOD_SECONDS = 0.01

# Audio buffer configuration - keep the last minute of audio
# Reachy produces ~20ms chunks, approximately 50 chunks per second
# 60 seconds × 50 chunks/second = 3000 chunks
AUDIO_BUFFER_MAX_SIZE = 3_000


class ReachyListensState(BaseModel):
    """Tracks the state for Reachy listening operations."""

    is_listening: bool = False
    is_recording: bool = False


class ReachyListens:
    """Handles listening operations for Reachy Mini."""

    def __init__(
        self,
        node: BaseNode,
        core_loop: asyncio.AbstractEventLoop,
        get_mini: Callable[[], Optional[ReachyMini]],
        on_audio_sample: Optional[Callable[[ReachyAudio], None]] = None,
    ):
        self._node = node
        self._core_loop = core_loop
        self._get_mini = get_mini
        self._on_audio_sample = on_audio_sample
        self._logger = node.get_logger()
        self._state = ReachyListensState()
        self._listening_timer: Optional[Timer] = None
        self._audio_buffer: Deque[npt.NDArray[np.float32]] = deque(maxlen=AUDIO_BUFFER_MAX_SIZE)
        self._logger.info("Reachy Listens initialized.")

    @property
    def state(self) -> ReachyListensState:
        return self._state

    def start_listening_timer(self) -> None:
        """Start the listening timer."""
        self._logger.info("Starting audio recording timer.")
        self._state.is_listening = True
        self._listening_timer = self._node.create_timer(AUDIO_POLL_PERIOD_SECONDS, self._poll_audio_sample)

    def stop_listening_timer(self) -> None:
        """Stop the listening timer."""
        self._logger.info("Stopping audio recording timer.")
        self._state.is_listening = False
        if self._listening_timer is not None:
            self._listening_timer.cancel()
            self._listening_timer = None

    def _poll_audio_sample(self) -> None:
        mini = self._get_mini()
        if mini is None:
            return

        try:
            sample: npt.NDArray[np.float32] = mini.media.get_audio_sample()
            if sample is not None:
                self._audio_buffer.append(sample)
                if self._on_audio_sample is not None:
                    reachy_audio = self._prepare_audio(mini, sample)
                    self._on_audio_sample(reachy_audio)
        except Exception as e:
            self._logger.error(f"Failed to get audio sample: {e}")

    def start_recording(self) -> None:
        """Start recording audio."""
        if self._state.is_recording:
            self._logger.warning("Already recording.")
            return

        # Once connected, we are always listening. Start recording simply clears the buffer so that we only capture
        # fresh samples for the given start-stop period.
        self._logger.info("Starting recording.")
        self._audio_buffer.clear()
        self._state.is_recording = True

    def stop_recording(self) -> Optional[ReachyAudio]:
        """Stop recording and return the audio data as a ReachyAudio object."""
        mini = self._get_mini()
        if mini is None:
            self._logger.warning("Reachy Mini not connected.")
            return None

        if not self._state.is_recording:
            self._logger.warning("Not currently recording.")
            return None

        self._logger.info("Stopping recording.")
        self._state.is_recording = False
        if len(self._audio_buffer) == 0:
            self._logger.warning("No audio samples collected.")
            return None

        audio_data = np.concatenate(list(self._audio_buffer))
        return self._prepare_audio(mini, audio_data)

    @staticmethod
    def _prepare_audio(mini: ReachyMini, sample: np.ndarray) -> ReachyAudio:
        # See: https://github.com/pollen-robotics/reachy_mini/blob/develop/examples/debug/sound_record.py
        channels = mini.media.get_input_channels()  # 2
        sample_rate = mini.media.get_input_audio_samplerate()  # 16000
        sample_width = sample.dtype.itemsize  # float32 = 4 bytes per sample

        # Transpose from (samples, channels) to (channels, samples) for AudioManager/librosa
        sample_transposed = sample.T if channels > 1 else sample
        audio_data = AudioManager.to_bytes(
            audio_array=sample_transposed,
            sample_width=sample_width,
            channels=channels,
        )

        # Done
        return ReachyAudio(
            audio_data=bytes(audio_data),
            sample_rate=sample_rate,
            channels=channels,
            sample_width=sample_width,
        )
