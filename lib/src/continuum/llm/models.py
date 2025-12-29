from pydantic import BaseModel

from continuum.models import ContinuumResponse, ContinuumRequest, ContinuumStreamingResponse


class ContinuumLlmRequest(ContinuumRequest):
    state_id: str = ""
    system_prompt: str = ""
    content_text: str


#
# Match the definitions in
# ./workspace/src/continuum_interfaces/msg/LlmRequest.msg
# ./workspace/src/continuum_interfaces/msg/LlmResponse.msg
# ./workspace/src/continuum_interfaces/msg/LlmStreamingResponse.msg
#


class ContinuumLlmResponse(ContinuumResponse):
    state_id: str = ""
    content_text: str = ""
    done_reason: str = ""


class ContinuumLlmStreamingResponse(ContinuumStreamingResponse):
    content_text: str = ""


#
# Provider-specific models
#


class BaseLlmOptions(BaseModel):
    """Base options class for all LLM providers."""

    model_name: str = ""


class FakeLlmOptions(BaseLlmOptions):
    error_rate: float = 0.25
    streaming_delay_seconds: float = 1.0


class OllamaLlmOptions(BaseLlmOptions):
    host: str = "http://localhost:11434"


class GoogleLlmOptions(BaseLlmOptions):
    api_key: str = ""


class OpenAiLlmOptions(BaseLlmOptions):
    api_key: str = ""
    base_url: str = ""
