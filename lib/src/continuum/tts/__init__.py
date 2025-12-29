"""TTS (Text-to-Speech) module for Continuum."""

from continuum.tts.models import (
    ContinuumTtsRequest,
    ContinuumTtsResponse,
    ContinuumTtsStreamingResponse,
    BaseTtsOptions,
    KokoroTtsOptions,
)
from continuum.tts.tts_client import ContinuumTtsClient
from continuum.tts.kokoro_tts_client import KokoroTtsClient

__all__ = [
    "ContinuumTtsRequest",
    "ContinuumTtsResponse",
    "ContinuumTtsStreamingResponse",
    "BaseTtsOptions",
    "KokoroTtsOptions",
    "ContinuumTtsClient",
    "KokoroTtsClient",
]
