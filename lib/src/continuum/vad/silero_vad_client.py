from typing import Optional, Callable

import numpy as np

from continuum.audio.constants import AUDIO_SAMPLE_RATE_16KHZ, AUDIO_FRAME_LENGTH_512
from continuum.vad.models import SileroVadOptions, ContinuumVadResponse, ContinuumVadStreamingResponse, VadEvent
from continuum.vad.vad_client import ContinuumVadClient


class SileroVadClient(ContinuumVadClient):
    """Silero VAD client."""

    def __init__(
        self,
        options: SileroVadOptions = SileroVadOptions(),
        streaming_callback: Optional[Callable[[ContinuumVadStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the VAD client."""
        super().__init__(streaming_callback)

        try:
            from silero_vad import load_silero_vad, VADIterator
        except ImportError as e:
            self._logger.error(
                "Failed to import Silero VAD library. "
                "Please install it with `pip install continuum[vad]` or equivalent."
            )
            raise e

        model = load_silero_vad(onnx=options.use_onnx)
        self._vad_iterator = VADIterator(
            model=model,
            sampling_rate=AUDIO_SAMPLE_RATE_16KHZ,
            threshold=options.threshold,
            min_silence_duration_ms=options.min_silence_duration_ms,
        )

        self._logger.info(f"Silero VAD client initialized: {options}.")

    def _reset_state(self) -> None:
        self._vad_iterator.reset_states()

    async def _execute_request(self, audio_array: np.ndarray) -> ContinuumVadResponse:
        # Silero expects exactly 512 samples per window (at 16 kHz = 32ms)
        for chunk in self._buffer_and_yield(audio_array, AUDIO_FRAME_LENGTH_512):
            result = self._vad_iterator(chunk)
            if result is not None and self.streaming_callback is not None:
                if result.get("start") is not None:
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_START)
                    )
                elif result.get("end") is not None:
                    self.streaming_callback(
                        ContinuumVadStreamingResponse(session_id=self._session_id, event=VadEvent.SPEECH_END)
                    )
                else:
                    self._logger.warning(f"Unexpected VAD result: {result}")

        return ContinuumVadResponse(session_id=self._session_id)
