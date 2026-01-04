import logging
import typing
from typing import Callable, Optional

from elevenlabs.client import ElevenLabs, OMIT

from continuum.constants import (
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
    DEFAULT_AUDIO_FORMAT,
)
from continuum.tts.models import (
    ContinuumTtsResponse,
    ContinuumTtsRequest,
    ContinuumTtsStreamingResponse,
    ElevenLabsTtsOptions,
)
from continuum.tts.tts_client import ContinuumTtsClient
from continuum.utils import none_if_empty


class ElevenLabsTtsClient(ContinuumTtsClient):
    """ElevenLabs TTS client."""

    def __init__(self, options: ElevenLabsTtsOptions = ElevenLabsTtsOptions()) -> None:
        """Initialize the ElevenLabs TTS client."""
        self._logger = logging.getLogger(__name__)
        self._client = ElevenLabs(api_key=options.api_key)
        self._options = options

    async def execute_request(
        self,
        request: ContinuumTtsRequest,
        streaming_callback: Optional[Callable[[ContinuumTtsStreamingResponse], None]] = None,
    ) -> ContinuumTtsResponse:
        """Generate audio data from text using ElevenLabs."""
        language = none_if_empty(request.language) or OMIT
        audio: typing.Iterator[bytes] = self._client.text_to_speech.convert(
            text=request.text,
            optimize_streaming_latency=1,  # 0: default, 1: normal, 2: strong, 3: max, 4: best
            output_format="pcm_16000",
            language_code=language,
            voice_id=self._options.voice_id,
            model_id=self._options.model_name,
        )

        # Collect audio chunks
        audio_chunks = []
        for chunk in audio:
            audio_chunks.append(chunk)

        # Combine all chunks and publish
        audio_data = b"".join(audio_chunks)
        response = ContinuumTtsResponse(
            session_id=request.session_id,
            is_initial=request.is_initial,
            is_final=request.is_final,
            order_id=request.order_id,
            format=DEFAULT_AUDIO_FORMAT,
            channels=DEFAULT_AUDIO_CHANNELS,
            sample_rate=DEFAULT_AUDIO_SAMPLE_RATE,
            sample_width=DEFAULT_AUDIO_SAMPLE_WIDTH,
        )
        response.set_audio_bytes(audio_data)
        return response

    def shutdown(self) -> None:
        """Shutdown the TTS client and clean up resources."""
        pass
