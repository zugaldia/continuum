from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse


class ContinuumAsrRequest(ContinuumRequest):
    audio_path: str
    language: str = ""


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


class BaseAsrOptions(BaseModel):
    """Base options class for all ASR providers."""

    model_name: str = ""


class FakeAsrOptions(BaseAsrOptions):
    error_rate: float = 0.25
    streaming_delay_seconds: float = 1.0


class FasterWhisperAsrOptions(BaseAsrOptions):
    device: str = "auto"
    download_root: str = ""  # If empty, use the standard Hugging Face cache directory


class OpenAiAsrOptions(BaseAsrOptions):
    api_key: str = ""
    base_url: str = ""
