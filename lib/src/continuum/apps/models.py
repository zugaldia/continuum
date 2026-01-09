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
