from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse


class ContinuumTtsRequest(ContinuumRequest):
    text: str
    language: str = ""


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/TtsRequest.msg
# ./workspace/src/continuum_interfaces/msg/TtsResponse.msg
# ./workspace/src/continuum_interfaces/msg/TtsStreamingResponse.msg
#


class ContinuumTtsResponse(ContinuumResponse):
    audio_path: str = ""


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
