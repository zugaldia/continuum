from collections import OrderedDict

from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest


class ContinuumAsrRequest(ContinuumRequest):
    session_id: str
    audio_path: str

    @classmethod
    def from_ros(cls, msg: OrderedDict) -> "ContinuumAsrRequest":
        return cls.model_validate(msg)


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/AsrRequest.msg
# ./workspace/src/continuum_interfaces/msg/AsrResponse.msg
#

class ContinuumAsrResponse(ContinuumResponse):
    session_id: str
    transcription: str


#
# Provider-specific models
#

class FasterWhisperOptions(BaseModel):
    """Configuration options for the Faster Whisper client."""

    model_size_or_path: str = "medium"
    device: str = "auto"
