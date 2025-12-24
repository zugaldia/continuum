import logging
from typing import Optional, Callable

from openai import OpenAI
from openai.types.audio import TranscriptionTextDoneEvent, TranscriptionTextDeltaEvent

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.models import (
    ContinuumAsrResponse,
    ContinuumAsrRequest,
    ContinuumAsrStreamingResponse,
    OpenAiAsrOptions,
)
from continuum.constants import ERROR_CODE_UNEXPECTED
from continuum.utils import none_if_empty


class OpenAiAsrClient(ContinuumAsrClient):
    """OpenAI ASR client."""

    def __init__(self, options: OpenAiAsrOptions = OpenAiAsrOptions()) -> None:
        """Initialize the OpenAI ASR client."""
        self._logger = logging.getLogger(__name__)
        self._client = OpenAI(api_key=none_if_empty(options.api_key), base_url=none_if_empty(options.base_url))

    async def execute_request(
        self,
        request: ContinuumAsrRequest,
        streaming_callback: Optional[Callable[[ContinuumAsrStreamingResponse], None]] = None,
    ) -> ContinuumAsrResponse:
        """Transcribe audio data to text using OpenAI."""
        with open(request.audio_path, "rb") as audio_file:
            events = self._client.audio.transcriptions.create(model="gpt-4o-transcribe", file=audio_file, stream=True)

        done_event: Optional[TranscriptionTextDoneEvent] = None
        for event in events:
            if isinstance(event, TranscriptionTextDoneEvent):
                done_event = event
            elif isinstance(event, TranscriptionTextDeltaEvent):
                if streaming_callback:
                    streaming_callback(
                        ContinuumAsrStreamingResponse(session_id=request.session_id, transcription=event.delta)
                    )
            else:
                self._logger.warning(f"Unknown event type: {event}")

        if done_event:
            return ContinuumAsrResponse(session_id=request.session_id, transcription=done_event.text)
        else:
            return ContinuumAsrResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No done event received.",
            )

    def shutdown(self) -> None:
        """Shutdown the ASR client and clean up resources."""
        pass
