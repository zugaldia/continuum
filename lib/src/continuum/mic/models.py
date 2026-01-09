from enum import StrEnum

from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse, AudioComponent


class MicAction(StrEnum):
    START = "START"
    STOP = "STOP"
    MUTE = "MUTE"
    UNMUTE = "UNMUTE"


class MicState(StrEnum):
    IDLE = "IDLE"
    STARTED = "STARTED"
    STARTED_MUTED = "STARTED_MUTED"


class ContinuumMicRequest(ContinuumRequest):
    action: MicAction


class ContinuumMicResponse(ContinuumResponse, AudioComponent):
    state: MicState


class ContinuumMicStreamingResponse(ContinuumStreamingResponse, AudioComponent):
    pass


#
# Provider-specific models
#


class BaseMicOptions(BaseModel):
    pass


class PyAudioMicOptions(BaseModel):
    pass


class PicovoiceMicOptions(BaseModel):
    pass


class GStreamerMicOptions(BaseModel):
    pass
