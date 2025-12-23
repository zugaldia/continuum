from continuum.models import (
    ContinuumResponse,
    ContinuumRequest,
    ContinuumStreamingResponse,
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


class ContinuumDictationRequest(ContinuumAppRequest):
    asr_node: str  # ASR node implementation to use (e.g., "faster_whisper", "fake")
    llm_node: str  # LLM node implementation to use (e.g., "ollama", "claude")
    audio_path: str


class ContinuumDictationResponse(ContinuumAppResponse):
    content_text: str = ""
    status: str = ""


class ContinuumDictationStreamingResponse(ContinuumStreamingResponse):
    pass
