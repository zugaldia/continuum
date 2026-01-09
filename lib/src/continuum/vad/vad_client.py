import logging
from abc import ABC, abstractmethod
from typing import Optional, Callable, Iterator

import numpy as np

from continuum.audio.audio_manager import AudioManager
from continuum.audio.constants import (
    AUDIO_FRAME_LENGTH_512,
    AUDIO_CHANNELS_MONO,
    AUDIO_SAMPLE_RATE_16KHZ,
    AUDIO_FORMAT_PCM,
)
from continuum.constants import ERROR_CODE_UNEXPECTED
from continuum.models import ContinuumExecutor
from continuum.vad.models import ContinuumVadRequest, ContinuumVadResponse, ContinuumVadStreamingResponse


class ContinuumVadClient(ContinuumExecutor, ABC):
    def __init__(self, streaming_callback: Optional[Callable[[ContinuumVadStreamingResponse], None]] = None) -> None:
        super().__init__(streaming_callback)
        self._logger = logging.getLogger(self.__class__.__name__)

        # Warm up librosa to avoid lazy initialization overhead on the first request
        dummy_audio = np.zeros(AUDIO_FRAME_LENGTH_512, dtype=np.float32)
        _ = AudioManager.get_rms(dummy_audio, frame_length=AUDIO_FRAME_LENGTH_512)

        self._session_id: Optional[str] = None
        self._audio_buffer: np.ndarray = np.array([], dtype=np.float32)

    async def execute_request(self, request: ContinuumVadRequest) -> ContinuumVadResponse:
        if request.format != AUDIO_FORMAT_PCM:
            return ContinuumVadResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="Unsupported audio format. Only PCM is supported.",
            )

        audio_array: np.ndarray = AudioManager.from_bytes(
            audio_data=request.audio_data, sample_width=request.sample_width, channels=request.channels
        )

        if request.sample_rate != AUDIO_SAMPLE_RATE_16KHZ:
            audio_array = AudioManager.resample(
                audio_array=audio_array,
                original_sample_rate=request.sample_rate,
                target_sample_rate=AUDIO_SAMPLE_RATE_16KHZ,
            )

        if request.channels != AUDIO_CHANNELS_MONO:
            audio_array = AudioManager.to_mono(audio_array=audio_array)

        if self._session_id is None or self._session_id != request.session_id:
            self._logger.info(f"New session detected: {request.session_id}")
            self._session_id = request.session_id
            self._audio_buffer = np.array([], dtype=np.float32)
            self._reset_state()

        return await self._execute_request(audio_array)

    def _buffer_and_yield(self, audio_array: np.ndarray, frame_length: int) -> Iterator[np.ndarray]:
        """Append audio to the internal buffer and yield complete chunks of the specified frame length."""
        self._audio_buffer = np.concatenate([self._audio_buffer, audio_array])
        while len(self._audio_buffer) >= frame_length:
            chunk = self._audio_buffer[:frame_length]
            self._audio_buffer = self._audio_buffer[frame_length:]
            yield chunk

    @abstractmethod
    def _reset_state(self) -> None:
        pass

    @abstractmethod
    async def _execute_request(self, audio_array: np.ndarray) -> ContinuumVadResponse:
        pass
