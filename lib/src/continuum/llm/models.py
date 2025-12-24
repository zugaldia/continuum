from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse


class ContinuumLlmRequest(ContinuumRequest):
    content_text: str


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/LlmRequest.msg
# ./workspace/src/continuum_interfaces/msg/LlmResponse.msg
# ./workspace/src/continuum_interfaces/msg/LlmStreamingResponse.msg
#


class ContinuumLlmResponse(ContinuumResponse):
    content_text: str = ""
    done_reason: str = ""


class ContinuumLlmStreamingResponse(ContinuumStreamingResponse):
    content_text: str = ""


#
# Provider-specific models
#


class FakeLlmOptions(BaseModel):
    error_rate: float = 0.25
    streaming_delay_seconds: float = 1.0


class OllamaLlmOptions(BaseModel):
    host: str = "http://localhost:11434"


class GoogleLlmOptions(BaseModel):
    api_key: str = ""


class OpenAiLlmOptions(BaseModel):
    api_key: str = ""
    base_url: str = ""
