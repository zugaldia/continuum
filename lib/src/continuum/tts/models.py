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
