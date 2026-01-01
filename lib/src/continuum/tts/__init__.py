"""TTS (Text-to-Speech) module for Continuum."""

from continuum.tts.elevenlabs_tts_client import ElevenLabsTtsClient
from continuum.tts.models import (
    ContinuumTtsRequest,
    ContinuumTtsResponse,
    ContinuumTtsStreamingResponse,
    BaseTtsOptions,
    KokoroTtsOptions,
    ElevenLabsTtsOptions,
)
from continuum.tts.tts_client import ContinuumTtsClient
from continuum.tts.kokoro_tts_client import KokoroTtsClient

__all__ = [
    "BaseTtsOptions",
    "ContinuumTtsClient",
    "ContinuumTtsRequest",
    "ContinuumTtsResponse",
    "ContinuumTtsStreamingResponse",
    "ElevenLabsTtsClient",
    "ElevenLabsTtsOptions",
    "KokoroTtsClient",
    "KokoroTtsOptions",
]
