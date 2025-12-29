import asyncio
import threading
from collections import deque
from pathlib import Path
from typing import Callable
from typing import Optional, Deque

import soundfile as sf
from pydantic import BaseModel
from rclpy.node import Node
from rclpy.timer import Timer
from reachy_mini import ReachyMini


# Audio playback configuration
AUDIO_PLAYBACK_CHUNK_SIZE = 1024


class ReachySpeaksState(BaseModel):
    """Tracks the state for Reachy speaking operations."""

    is_speaking: bool = False


class ReachySpeaks:
    """Handles speaking operations for Reachy Mini."""

    def __init__(
        self,
        node: Node,
        core_loop: asyncio.AbstractEventLoop,
        get_mini: Callable[[], Optional[ReachyMini]],
        on_queue_empty: Optional[Callable[[], None]] = None,
    ):
        self._node = node
        self._core_loop = core_loop
        self._get_mini = get_mini
        self._on_queue_empty = on_queue_empty
        self._logger = node.get_logger()
        self._state = ReachySpeaksState()
        self._playback_timer: Optional[Timer] = None
        self._tts_queue: Deque[Path] = deque()
        self._tts_queue_lock: threading.Lock = threading.Lock()
        self._logger.info("Reachy Speaks initialized.")

    @property
    def state(self) -> ReachySpeaksState:
        return self._state

    def clear_tts_queue(self) -> None:
        """Clear all queued TTS responses."""
        with self._tts_queue_lock:
            if self._tts_queue:
                self._logger.info(f"Clearing TTS queue with {len(self._tts_queue)} pending responses")
                self._tts_queue.clear()

    def play_audio_queued(self, audio_path: Path) -> None:
        with self._tts_queue_lock:
            # If not currently playing, play immediately
            if not self._state.is_speaking:
                self._play_audio(audio_path)
            else:
                # Otherwise, queue for later playback
                self._tts_queue.append(audio_path)
                self._logger.info(f"TTS response queued. Queue size: {len(self._tts_queue)}")

    def _play_audio(self, audio_path: Path) -> None:
        """Play audio from a file path."""
        mini = self._get_mini()
        if mini is None:
            self._logger.warning("Reachy Mini not connected.")
            return

        if self._state.is_speaking:
            self._logger.warning("Already speaking.")
            return

        data, sample_rate_file = sf.read(audio_path, dtype="float32")
        sample_rate_output = mini.media.get_output_audio_samplerate()  # 16000
        if sample_rate_file != sample_rate_output:
            self._logger.warning(f"Sampling mismatch: {sample_rate_file} != {sample_rate_output}.")
            return
        if data.ndim > 1:
            self._logger.warning("Audio is multichannel, mono is expected.")
            return

        duration = len(data) / sample_rate_output
        self._logger.info(f"Playing {duration} seconds of audio from: {audio_path}")

        self._state.is_speaking = True
        mini.media.start_playing()
        for i in range(0, len(data), AUDIO_PLAYBACK_CHUNK_SIZE):
            chunk = data[i : i + AUDIO_PLAYBACK_CHUNK_SIZE]
            mini.media.push_audio_sample(chunk)

        # We need to wait for the duration of the audio before calling stop_playing.
        # Otherwise, playback will be interrupted.
        self._playback_timer = self._node.create_timer(duration, self._finish_audio_playback)

    def _finish_audio_playback(self) -> None:
        """Finish audio playback and clean up the one-shot timer."""
        if self._playback_timer is not None:
            self._playback_timer.cancel()
            self._playback_timer = None

        mini = self._get_mini()
        if mini is not None:
            mini.media.stop_playing()
            self._logger.info("Audio played.")

        self._state.is_speaking = False

        # Process the TTS queue
        with self._tts_queue_lock:
            if self._tts_queue:
                next_audio_path = self._tts_queue.popleft()
                self._logger.info(f"Playing next queued TTS response. Remaining queue size: {len(self._tts_queue)}")
                self._play_audio(next_audio_path)
            else:
                self._logger.info("TTS queue empty, session complete")
                if self._on_queue_empty:
                    self._on_queue_empty()
