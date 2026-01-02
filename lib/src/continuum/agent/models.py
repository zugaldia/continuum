from enum import StrEnum
from typing import Callable

from pydantic import BaseModel

from continuum.agent.agent_tools import get_date_and_time
from continuum.models import ContinuumRequest, ContinuumResponse, ContinuumStreamingResponse


class AgentProvider(StrEnum):
    ANTHROPIC = "anthropic"
    GOOGLE = "google"
    OLLAMA = "ollama"
    OPENAI = "openai"


class ContinuumAgentRequest(ContinuumRequest):
    state_id: str = ""
    content_text: str


class ContinuumAgentResponse(ContinuumResponse):
    state_id: str = ""
    content_text: str = ""


class ContinuumAgentStreamingResponse(ContinuumStreamingResponse):
    pass


class BaseAgentOptions(BaseModel):
    provider_name: str = AgentProvider.OLLAMA
    model_name: str = "gpt-oss"
    api_key: str = ""
    base_url: str = "http://localhost:11434/v1"
    instructions: str = "You are a helpful assistant."
    enable_web_search_tool: bool = False
    enable_web_fetch_tool: bool = False
    enable_memory_tool: bool = False
    enable_file_search_tool: bool = False
    tools: list[Callable] = [get_date_and_time]


class PydanticAgentOptions(BaseAgentOptions):
    pass
