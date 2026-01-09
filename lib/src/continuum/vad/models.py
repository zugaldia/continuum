from enum import StrEnum

from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse, AudioComponent


class VadEvent(StrEnum):
    SPEECH_START = "SPEECH_START"
    SPEECH_END = "SPEECH_END"


class ContinuumVadRequest(ContinuumRequest, AudioComponent):
    pass


class ContinuumVadResponse(ContinuumResponse):
    pass


class ContinuumVadStreamingResponse(ContinuumStreamingResponse):
    event: VadEvent


#
# Provider-specific models
#


class BaseVadOptions(BaseModel):
    pass


class NaiveVadOptions(BaseVadOptions):
    rms_threshold: float = 0.1
    rms_gain: float = 10.0
    min_speech_frames: int = 3  # Minimum consecutive frames above the threshold before SPEECH_START (~96ms)
    min_silence_frames: int = 5  # Minimum consecutive frames below the threshold before SPEECH_END (~160ms)


class SileroVadOptions(BaseVadOptions):
    threshold: float = 0.5
    min_silence_duration_ms: int = 100
    use_onnx: bool = False


class PicovoiceVadOptions(BaseVadOptions):
    api_key: str = ""
    device: str = ""
    library_path: str = ""
    probability_threshold: float = 0.8
