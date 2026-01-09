import struct
from typing import Optional, Callable

import numpy as np

from continuum.audio.audio_manager import AudioManager
from continuum.audio.constants import AUDIO_SAMPLE_WIDTH_16BIT, AUDIO_CHANNELS_MONO
from continuum.utils import none_if_empty
from continuum.vad.models import (
    PicovoiceVadOptions,
    ContinuumVadResponse,
    ContinuumVadStreamingResponse,
    VadEvent,
)
from continuum.vad.vad_client import ContinuumVadClient


class PicovoiceVadClient(ContinuumVadClient):
    """Picovoice VAD client."""

    def __init__(
        self,
        options: PicovoiceVadOptions = PicovoiceVadOptions(),
        streaming_callback: Optional[Callable[[ContinuumVadStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the VAD client."""
        super().__init__(streaming_callback)

        try:
            import pvcobra
        except ImportError as e:
            self._logger.error(
                "Failed to import Picovoice Cobra VAD library. "
                "Please install it with `pip install continuum[vad]` or equivalent."
            )
            raise e

        library_path = none_if_empty(options.library_path)
        available_devices = pvcobra.available_devices(library_path=library_path)
        self._logger.info(f"Picovoice VAD available devices: {available_devices}")

        self._cobra: pvcobra.Cobra = pvcobra.create(
            access_key=options.api_key, device=none_if_empty(options.device), library_path=library_path
        )

        self._options = options
        self._is_speech_detected = False
        self._frame_length = self._cobra.frame_length
        self._logger.info(f"Picovoice VAD client v{self._cobra.version} initialized: {options}.")

    def shutdown(self):
        self._cobra.delete()

    def _reset_state(self) -> None:
        """Reset the VAD state for a new session."""
        self._is_speech_detected = False

    async def _execute_request(self, audio_array: np.ndarray) -> ContinuumVadResponse:
        for chunk in self._buffer_and_yield(audio_array, self._frame_length):
            audio_bytes = AudioManager.to_bytes(
                audio_array=chunk, sample_width=AUDIO_SAMPLE_WIDTH_16BIT, channels=AUDIO_CHANNELS_MONO
            )
            num_samples = len(audio_bytes) // AUDIO_SAMPLE_WIDTH_16BIT
            frame = struct.unpack("h" * num_samples, bytes(audio_bytes))
            probability: float = self._cobra.process(frame)
            if self.streaming_callback is not None:
                voice_detected = probability > self._options.probability_threshold
                if voice_detected and not self._is_speech_detected:
                    self._is_speech_detected = True
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_START)
                    )
                elif not voice_detected and self._is_speech_detected:
                    self._is_speech_detected = False
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_END)
                    )

        return ContinuumVadResponse(session_id=self._session_id)
