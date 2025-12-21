import logging
from typing import Iterable

from faster_whisper import WhisperModel
from faster_whisper.transcribe import Segment, TranscriptionInfo

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.models import ContinuumAsrResponse, ContinuumAsrRequest, FasterWhisperOptions


class FasterWhisperClient(ContinuumAsrClient):
    """Faster Whisper ASR (Automatic Speech Recognition) client."""

    def __init__(self, options: FasterWhisperOptions = FasterWhisperOptions()) -> None:
        """Initialize the Faster Whisper ASR client."""
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._model = WhisperModel(options.model_size_or_path, device=options.device)
        self._logger.info(f"Faster Whisper ASR client initialized with: {options}")

    async def execute_request(self, request: ContinuumAsrRequest) -> ContinuumAsrResponse:
        """Transcribe audio data to text using Faster Whisper."""
        self._logger.info(f"Starting transcription for request: {request}")

        segments: Iterable[Segment]
        info: TranscriptionInfo
        segments, info = self._model.transcribe(request.audio_path)
        self._logger.info(f"Transcription language: {info.language} ({info.language_probability})")

        segments_executed = list(segments)  # segments is a generator
        self._logger.info(f"Transcribed segments: {segments_executed}")

        transcription_text = " ".join([segment.text for segment in segments_executed])
        self._logger.info(f"Transcription: {transcription_text}")

        response = ContinuumAsrResponse(
            session_id=request.session_id,
            transcription=transcription_text
        )

        self._logger.info(f"Transcription completed for session_id: {request.session_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the ASR client and clean up resources."""
        self._logger.info("Faster Whisper ASR client shutting down.")
