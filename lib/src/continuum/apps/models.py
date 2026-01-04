import base64
from typing import Any, Dict

from continuum.models import (
    ContinuumResponse,
    ContinuumRequest,
    ContinuumStreamingResponse,
    AudioComponent,
)

# Dictation status constants
DICTATION_STATUS_QUEUED = "queued"
DICTATION_STATUS_TRANSCRIBED = "transcribed"
DICTATION_STATUS_COMPLETED = "completed"


class ContinuumAppRequest(ContinuumRequest):
    pass


class ContinuumAppResponse(ContinuumResponse):
    pass


#
# Dictation app
#


class ContinuumDictationRequest(ContinuumAppRequest, AudioComponent):
    language: str = ""
    context: str = ""

    @classmethod
    def from_ros(cls, msg: Dict[str, Any]) -> "ContinuumDictationRequest":
        """Override to handle base64-encoded audio_data from rosbridge."""
        # rosbridge encodes uint8[] as base64 strings
        if "audio_data" in msg and isinstance(msg["audio_data"], str):
            # Decode base64 string to bytes, then convert to list[int]
            audio_bytes = base64.b64decode(msg["audio_data"])
            msg["audio_data"] = list(audio_bytes)
        return cls.model_validate(msg)


class ContinuumDictationResponse(ContinuumAppResponse):
    content_text: str = ""
    status: str = ""
    asr_node: str = ""
    llm_node: str = ""
    asr_duration_seconds: float = 0.0
    llm_duration_seconds: float = 0.0
    total_duration_seconds: float = 0.0


class ContinuumDictationStreamingResponse(ContinuumStreamingResponse):
    pass
