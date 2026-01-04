import base64
from typing import Any, Dict

from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse, AudioComponent


class ContinuumTtsRequest(ContinuumRequest):
    is_initial: bool = False
    is_final: bool = False
    order_id: int = 0
    text: str
    language: str = ""


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/TtsRequest.msg
# ./workspace/src/continuum_interfaces/msg/TtsResponse.msg
# ./workspace/src/continuum_interfaces/msg/TtsStreamingResponse.msg
#


class ContinuumTtsResponse(ContinuumResponse, AudioComponent):
    is_initial: bool = False
    is_final: bool = False
    order_id: int = 0

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumTtsResponse":
        """Override to handle base64-encoded audio_data from rosbridge."""
        # rosbridge encodes uint8[] as base64 strings
        if "audio_data" in msg and isinstance(msg["audio_data"], str):
            # Decode base64 string to bytes, then convert to list[int]
            audio_bytes = base64.b64decode(msg["audio_data"])
            msg["audio_data"] = list(audio_bytes)
        return cls.model_validate(msg)


class ContinuumTtsStreamingResponse(ContinuumStreamingResponse):
    audio_chunk: bytes = b""


#
# Provider-specific models
#


class BaseTtsOptions(BaseModel):
    """Base options class for all TTS providers."""

    model_name: str = ""


class KokoroTtsOptions(BaseTtsOptions):
    device: str = ""


class ElevenLabsTtsOptions(BaseTtsOptions):
    api_key: str = ""
    voice_id: str = ""
