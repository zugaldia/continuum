import logging
from typing import Iterable, Optional, Callable, List

from faster_whisper import WhisperModel, available_models
from faster_whisper.transcribe import Segment, TranscriptionInfo

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.models import (
    ContinuumAsrResponse,
    ContinuumAsrRequest,
    FasterWhisperAsrOptions,
    ContinuumAsrStreamingResponse,
)
from continuum.utils import none_if_empty


class FasterWhisperAsrClient(ContinuumAsrClient):
    """Faster Whisper ASR (Automatic Speech Recognition) client."""

    def __init__(self, options: FasterWhisperAsrOptions = FasterWhisperAsrOptions()) -> None:
        """Initialize the Faster Whisper ASR client."""
        self._logger = logging.getLogger(__name__)
        self._model = WhisperModel(
            model_size_or_path=options.model_size_or_path,
            device=options.device,
            download_root=none_if_empty(options.download_root),
        )

    @staticmethod
    def get_available_models() -> List[str]:
        return available_models()

    def get_supported_languages(self) -> List[str]:
        return self._model.supported_languages

    async def execute_request(
        self,
        request: ContinuumAsrRequest,
        streaming_callback: Optional[Callable[[ContinuumAsrStreamingResponse], None]] = None,
    ) -> ContinuumAsrResponse:
        """Transcribe audio data to text using Faster Whisper."""
        segments: Iterable[Segment]
        info: TranscriptionInfo
        segments, info = self._model.transcribe(request.audio_path)
        self._logger.info(f"Language: {info.language} (confidence: {info.language_probability:.2f})")

        # Process segments and stream intermediate results
        outputs: List[Segment] = []
        for segment in segments:
            outputs.append(segment)
            if streaming_callback:
                streaming_callback(
                    ContinuumAsrStreamingResponse(session_id=request.session_id, transcription=segment.text)
                )

        # Prepare the final response
        transcription_text = "".join([segment.text for segment in outputs])
        response = ContinuumAsrResponse(
            session_id=request.session_id,
            transcription=transcription_text,
            language=info.language,
            language_probability=info.language_probability,
        )

        return response

    def shutdown(self) -> None:
        """Shutdown the ASR client and clean up resources."""
        pass
