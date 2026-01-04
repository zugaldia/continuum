import logging
from io import BytesIO
from typing import Callable, Generator, Optional, Dict

import numpy
import soundfile as sf
import torch
from kokoro import KPipeline
from scipy import signal

from continuum.constants import (
    ERROR_CODE_UNEXPECTED,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
    DEFAULT_AUDIO_FORMAT,
)
from continuum.tts.models import (
    ContinuumTtsResponse,
    ContinuumTtsRequest,
    ContinuumTtsStreamingResponse,
    KokoroTtsOptions,
)
from continuum.tts.tts_client import ContinuumTtsClient
from continuum.utils import none_if_empty, is_empty

# Kokoro's native sample rate (hardcoded in the model, cannot be changed)
KOKORO_NATIVE_SAMPLE_RATE = 24000


class KokoroTtsClient(ContinuumTtsClient):
    """Kokoro TTS client."""

    LANGUAGE_CODE_MAP = {
        "en": "a",  # English (American)
        "es": "e",  # Spanish
        "fr": "f",  # French
        "hi": "h",  # Hindi
        "it": "i",  # Italian
        "pt": "p",  # Portuguese (Brazilian)
        "ja": "j",  # Japanese
        "zh": "z",  # Chinese (Mandarin)
    }

    def __init__(self, options: KokoroTtsOptions = KokoroTtsOptions()) -> None:
        """Initialize the Kokoro TTS client."""
        self._logger = logging.getLogger(__name__)
        self._pipelines: Dict[str, KPipeline] = {}
        self._options = options

    def _translate_language_code(self, iso_code: str) -> str:
        """Translate ISO language code to Kokoro's single-character format."""
        normalized_code = iso_code.lower().strip()
        if is_empty(normalized_code):
            return "a"  # Default to English (American)
        if normalized_code not in self.LANGUAGE_CODE_MAP:
            supported = ", ".join(sorted(self.LANGUAGE_CODE_MAP.keys()))
            raise ValueError(f"Language code '{iso_code}' not supported by Kokoro. Supported languages: {supported}")
        return self.LANGUAGE_CODE_MAP[normalized_code]

    def _get_pipeline(self, kokoro_language: str) -> KPipeline:
        if kokoro_language not in self._pipelines:
            self._pipelines[kokoro_language] = KPipeline(
                lang_code=kokoro_language,
                repo_id="hexgrad/Kokoro-82M",
                device=none_if_empty(self._options.device),  # None for auto
            )

        return self._pipelines[kokoro_language]

    async def execute_request(
        self,
        request: ContinuumTtsRequest,
        streaming_callback: Optional[Callable[[ContinuumTtsStreamingResponse], None]] = None,
    ) -> ContinuumTtsResponse:
        """Generate audio data from text using Kokoro."""
        kokoro_language = self._translate_language_code(request.language)
        self._logger.info(f"Using Kokoro language code '{kokoro_language}' (for ISO code: {request.language})")

        pipeline = self._get_pipeline(kokoro_language)

        # More voices here: https://huggingface.co/hexgrad/Kokoro-82M/blob/main/VOICES.md
        # `split_pattern=None` makes the audio generation do the full text in one go. We leave the responsibility to
        # split/chunk the text to TTS consumers (e.g. for lower latency).
        generator: Generator[KPipeline.Result, None, None] = pipeline(
            request.text, voice="af_heart", split_pattern=None
        )

        chunk_index: int
        graphemes: str
        phonemes: str
        audio: Optional[torch.FloatTensor]
        audio_data: Optional[bytes] = None
        for chunk_index, (graphemes, phonemes, audio) in enumerate(generator):
            self._logger.debug(f"Chunk {chunk_index}: {graphemes} -> {phonemes}")
            resampled_audio: numpy.ndarray = self._resample(audio.numpy())
            buffer = BytesIO()
            sf.write(buffer, resampled_audio, DEFAULT_AUDIO_SAMPLE_RATE, format="RAW", subtype="PCM_16")
            audio_data = buffer.getvalue()
            break  # With split_pattern=None, all text is processed as a single chunk

        if audio_data is None:
            return ContinuumTtsResponse(
                session_id=request.session_id,
                is_initial=request.is_initial,
                is_final=request.is_final,
                order_id=request.order_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No audio generated.",
            )
        else:
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

    @staticmethod
    def _resample(data: numpy.ndarray) -> numpy.ndarray:
        # Resample from 24kHz to 16kHz using scipy (3:2 ratio)
        # Using resample_poly for efficiency with integer ratios
        return signal.resample_poly(data, up=2, down=3)

    def shutdown(self) -> None:
        """Shutdown the TTS client and clean up resources."""
        pass
