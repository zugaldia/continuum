from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse


class ContinuumAsrRequest(ContinuumRequest):
    audio_path: str


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/AsrRequest.msg
# ./workspace/src/continuum_interfaces/msg/AsrResponse.msg
#

class ContinuumAsrResponse(ContinuumResponse):
    transcription: str = ""
    language: str = ""  # Empty string indicates language not detected/available
    language_probability: float = -1.0  # -1.0 indicates probability not available


class ContinuumAsrStreamingResponse(ContinuumStreamingResponse):
    transcription: str = ""


#
# Provider-specific models
#

class FasterWhisperOptions(BaseModel):
    """Configuration options for the Faster Whisper client."""

    model_size_or_path: str = "medium"
    device: str = "auto"
