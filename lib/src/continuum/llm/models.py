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


class OllamaOptions(BaseModel):
    """Configuration options for the Ollama client."""

    pass
