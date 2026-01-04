"""

This class is in charge of managing the audio queue. Not only can multiple audios be added at once, they might also
arrive in the wrong order, which is typical for streaming LLM responses.

"""

import asyncio
import io
import threading
from collections import deque
from typing import Callable
from typing import Optional, Deque

import soundfile as sf
from pydantic import BaseModel
from rclpy.timer import Timer
from reachy_mini import ReachyMini

from continuum_core.reachy.models import ReachyAudio
from continuum_core.shared.base_node import BaseNode

# Audio playback configuration
AUDIO_PLAYBACK_CHUNK_SIZE = 1024


class ReachySpeaksState(BaseModel):
    """Tracks the state for Reachy speaking operations."""

    is_speaking: bool = False
    audio_queue: Deque[ReachyAudio] = deque()  # Unified queue for all audio
    initial_received: bool = False  # Whether we've received the initial response in a sequence


class ReachySpeaks:
    """Handles speaking operations for Reachy Mini."""

    def __init__(
        self,
        node: BaseNode,
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
        self._audio_queue_lock: threading.Lock = threading.Lock()
        self._logger.info("Reachy Speaks initialized.")

    @property
    def state(self) -> ReachySpeaksState:
        return self._state

    def clear_audio_queue(self) -> None:
        """Clear all queued audio responses."""
        with self._audio_queue_lock:
            if self._state.audio_queue:
                self._logger.info(f"Clearing audio queue with {len(self._state.audio_queue)} pending responses")
                self._state.audio_queue.clear()
        self._state.initial_received = False

    def queue_audio(self, reachy_audio: ReachyAudio) -> None:
        """Queue an audio and play it when appropriate."""
        with self._audio_queue_lock:
            self._state.audio_queue.append(reachy_audio)
            queue_size = len(self._state.audio_queue)

        if reachy_audio.is_initial:
            self._state.initial_received = True
            self._logger.info("Initial audio response received")

        self._logger.info(
            f"Audio response queued (order_id={reachy_audio.order_id}, is_initial={reachy_audio.is_initial}). "
            f"Queue size: {queue_size}"
        )

        self._process_audio_queue()

    def _process_audio_queue(self) -> None:
        """Try to play the next audio response from the queue."""
        if not self._state.initial_received:
            self._logger.info("Waiting for initial audio response before playback")
            return
        if self._state.is_speaking:
            self._logger.info("Already speaking, will process queue after current audio finishes")
            return

        with self._audio_queue_lock:
            if len(self._state.audio_queue) == 0:
                self._logger.info("Audio queue is empty")
                return

            # Sort queue and play the first (lowest order_id) response. We play one at a time instead of triggering
            # all the remaining items here to give a chance to other out-of-order audios, for example, from a LLM or
            # TTS response to come in.
            sorted_queue = sorted(self._state.audio_queue, key=lambda r: r.order_id)
            next_audio = sorted_queue[0]
            self._state.audio_queue.remove(next_audio)
            remaining_size = len(self._state.audio_queue)

        self._logger.info(
            f"Playing audio response with order_id={next_audio.order_id} "
            f"(is_initial={next_audio.is_initial}). "
            f"Remaining queue size: {remaining_size}"
        )

        self._play_audio(next_audio)

    def _play_audio(self, reachy_audio: ReachyAudio) -> None:
        """Play audio from audio data bytes."""
        mini = self._get_mini()
        if mini is None:
            self._logger.warning("Reachy Mini not connected.")
            return
        if self._state.is_speaking:
            self._logger.warning("Already speaking.")
            return

        # Convert audio bytes to numpy array using soundfile (raw PCM format)
        audio_io = io.BytesIO(reachy_audio.audio_data)
        data, sample_rate_file = sf.read(
            audio_io,
            samplerate=reachy_audio.sample_rate,
            channels=reachy_audio.channels,
            format="RAW",
            subtype="PCM_16",
            dtype="float32",
        )

        sample_rate_output = mini.media.get_output_audio_samplerate()  # 16000
        if sample_rate_file != sample_rate_output:
            self._logger.warning(f"Sampling mismatch: {sample_rate_file} != {sample_rate_output}.")
            return
        if data.ndim > 1:
            self._logger.warning("Audio is multichannel, mono is expected.")
            return

        duration = len(data) / sample_rate_output
        self._logger.info(f"Playing {duration:.2f} seconds of audio ({len(reachy_audio.audio_data)} bytes)")

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
        with self._audio_queue_lock:
            has_audio = len(self._state.audio_queue) > 0
        if has_audio:
            self._process_audio_queue()
        else:
            self._logger.info("All audio playback complete")
            if self._on_queue_empty:
                self._on_queue_empty()
