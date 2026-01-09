from typing import Optional, Callable

import numpy as np

from continuum.audio.audio_manager import AudioManager
from continuum.audio.constants import AUDIO_FRAME_LENGTH_512
from continuum.vad.models import (
    NaiveVadOptions,
    ContinuumVadResponse,
    ContinuumVadStreamingResponse,
    VadEvent,
)
from continuum.vad.vad_client import ContinuumVadClient


class NaiveVadClient(ContinuumVadClient):
    """Naive VAD client."""

    def __init__(
        self,
        options: NaiveVadOptions = NaiveVadOptions(),
        streaming_callback: Optional[Callable[[ContinuumVadStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the VAD client."""
        super().__init__(streaming_callback)
        self._options = options
        self._is_speech_detected = False
        self._speech_frame_count = 0  # Consecutive frames with voice detected
        self._silence_frame_count = 0  # Consecutive frames without voice detected
        self._logger.info(f"Naive VAD client initialized: {self._options}")

    def _reset_state(self) -> None:
        """Reset the VAD state for a new session."""
        self._is_speech_detected = False
        self._speech_frame_count = 0
        self._silence_frame_count = 0

    async def _execute_request(self, audio_array: np.ndarray) -> ContinuumVadResponse:
        """Process audio through RMS-based VAD with debouncing to prevent false triggers from brief noise or silence."""
        for chunk in self._buffer_and_yield(audio_array, AUDIO_FRAME_LENGTH_512):
            chunk_rms = AudioManager.get_rms(chunk, frame_length=AUDIO_FRAME_LENGTH_512)
            chunk_rms_gained = chunk_rms * self._options.rms_gain
            voice_detected = chunk_rms_gained >= self._options.rms_threshold
            if voice_detected:
                self._speech_frame_count += 1
                self._silence_frame_count = 0
            else:
                self._silence_frame_count += 1
                self._speech_frame_count = 0

            # Trigger events (debounced)
            if self.streaming_callback is not None:
                if not self._is_speech_detected and self._speech_frame_count >= self._options.min_speech_frames:
                    self._is_speech_detected = True
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_START)
                    )
                elif self._is_speech_detected and self._silence_frame_count >= self._options.min_silence_frames:
                    self._is_speech_detected = False
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_END)
                    )

        return ContinuumVadResponse(session_id=self._session_id)
