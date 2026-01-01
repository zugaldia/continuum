import logging
import typing
import wave
from typing import Callable, Optional

from elevenlabs.client import ElevenLabs, OMIT

from continuum.tts.models import (
    ContinuumTtsResponse,
    ContinuumTtsRequest,
    ContinuumTtsStreamingResponse,
    ElevenLabsTtsOptions,
)
from continuum.tts.tts_client import ContinuumTtsClient
from continuum.utils import create_timestamped_filename, none_if_empty

# Target sample rate for output audio files
TARGET_SAMPLE_RATE = 16000


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

        # Combine all chunks
        audio_data = b"".join(audio_chunks)

        # Write as WAV file
        # ElevenLabs pcm_16000 format: 16kHz sample rate, 16-bit PCM, mono
        audio_path = create_timestamped_filename("elevenlabs", "wav")
        with wave.open(str(audio_path), "wb") as wav_file:
            wav_file.setnchannels(1)  # Mono
            wav_file.setsampwidth(2)  # 16-bit = 2 bytes
            wav_file.setframerate(TARGET_SAMPLE_RATE)
            wav_file.writeframes(audio_data)

        self._logger.info(f"Generated audio file: {audio_path}")
        return ContinuumTtsResponse(
            session_id=request.session_id,
            is_initial=request.is_initial,
            is_final=request.is_final,
            order_id=request.order_id,
            audio_path=str(audio_path),
        )

    def shutdown(self) -> None:
        """Shutdown the TTS client and clean up resources."""
        pass
