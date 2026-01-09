import logging
from typing import Optional, Callable, AsyncIterable

from pydantic_ai import (
    Agent,
    RunContext,
    AgentStreamEvent,
    ModelMessage,
    WebSearchTool,
    FinalResultEvent,
    PartStartEvent,
    FileSearchTool,
    MemoryTool,
    WebFetchTool,
    PartDeltaEvent,
    PartEndEvent,
    Tool,
    FunctionToolCallEvent,
    FunctionToolResultEvent,
)
from pydantic_ai.models.anthropic import AnthropicModel
from pydantic_ai.models.google import GoogleModel
from pydantic_ai.models.openai import OpenAIChatModel
from pydantic_ai.providers.anthropic import AnthropicProvider
from pydantic_ai.providers.google import GoogleProvider
from pydantic_ai.providers.ollama import OllamaProvider
from pydantic_ai.providers.openai import OpenAIProvider

from continuum.agent.agent_runner import AgentRunner
from continuum.agent.models import (
    PydanticAgentOptions,
    ContinuumAgentRequest,
    ContinuumAgentStreamingResponse,
    ContinuumAgentResponse,
    AgentProvider,
)
from continuum.constants import ERROR_CODE_UNEXPECTED
from continuum.utils import is_empty, none_if_empty


class PydanticAgentRunner(AgentRunner):
    def __init__(
        self,
        options: PydanticAgentOptions,
        streaming_callback: Optional[Callable[[ContinuumAgentStreamingResponse], None]] = None,
    ):
        super().__init__(streaming_callback)
        self._logger = logging.getLogger(__name__)
        self._options = options

        # For now, this is purely an in-memory history that works well while the node keeps the runner instance,
        # but it would not work, for example, when invoked from the CLI.
        self._history: dict[str, list[ModelMessage]] = {}

        model = self._get_model()
        tools = self._get_tools()
        builtin_tools = self._get_builtin_tools()
        self._logger.info(
            f"Loading {model.model_name} with {len(tools)} tools and {len(builtin_tools)} built-in tools."
        )

        self._agent = Agent(
            model=model, instructions=self._options.instructions, tools=tools, builtin_tools=builtin_tools
        )

    def _get_model(self):
        if self._options.provider_name == AgentProvider.ANTHROPIC:
            provider = AnthropicProvider(api_key=self._options.api_key)
            return AnthropicModel(self._options.model_name, provider=provider)
        elif self._options.provider_name == AgentProvider.GOOGLE:
            provider = GoogleProvider(api_key=self._options.api_key)
            return GoogleModel(self._options.model_name, provider=provider)
        elif self._options.provider_name == AgentProvider.OLLAMA:
            provider = OllamaProvider(
                base_url=none_if_empty(self._options.base_url),
                api_key=none_if_empty(self._options.api_key),
            )
            return OpenAIChatModel(self._options.model_name, provider=provider)
        elif self._options.provider_name == AgentProvider.OPENAI:
            provider = OpenAIProvider(api_key=self._options.api_key)
            return OpenAIChatModel(self._options.model_name, provider=provider)
        else:
            raise NotImplementedError(f"Provider is not supported: {self._options.provider_name}")

    def _get_tools(self):
        tools = [
            Tool(function=tool, takes_ctx=False, require_parameter_descriptions=True, requires_approval=False)
            for tool in self._options.tools
        ]
        return tools

    def _get_builtin_tools(self):
        builtin_tools = []
        if self._options.enable_web_search_tool:
            builtin_tools.append(WebSearchTool())
        if self._options.enable_web_fetch_tool:
            builtin_tools.append(WebFetchTool())
        if self._options.enable_memory_tool:
            builtin_tools.append(MemoryTool())
        if self._options.enable_file_search_tool:
            builtin_tools.append(FileSearchTool(file_store_ids=[]))
        return builtin_tools

    def _load_history(self, state_id: str) -> list[ModelMessage]:
        """Load message history for a given state ID."""
        if is_empty(state_id):
            return []
        message_history = self._history.get(state_id, [])
        self._logger.info(f"Loaded {len(message_history)} messages for state {state_id}")
        return message_history

    def _save_history(self, state_id: str, messages: list[ModelMessage]) -> None:
        """Save message history for a given state ID."""
        if is_empty(state_id):
            return
        self._history[state_id] = messages
        self._logger.info(f"Saved {len(messages)} messages for state {state_id}")

    async def execute_request(
        self,
        request: ContinuumAgentRequest,
    ) -> ContinuumAgentResponse:
        async def event_stream_handler(ctx: RunContext, event_stream: AsyncIterable[AgentStreamEvent]):
            async for event in event_stream:
                if isinstance(event, PartStartEvent):
                    self._logger.info(f"PartStartEvent: {event}")
                elif isinstance(event, PartDeltaEvent):
                    self._logger.info(f"PartDeltaEvent: {event}")
                elif isinstance(event, PartEndEvent):
                    self._logger.info(f"PartEndEvent: {event}")
                elif isinstance(event, FinalResultEvent):
                    self._logger.info(f"FinalResultEvent: {event}")
                elif isinstance(event, FunctionToolCallEvent):
                    self._logger.info(f"FunctionToolCallEvent: {event}")
                elif isinstance(event, FunctionToolResultEvent):
                    self._logger.info(f"FunctionToolResultEvent: {event}")
                else:
                    self._logger.warning(f"Unrecognized event: {event}")

        user_prompt = request.content_text
        message_history = self._load_history(request.state_id)
        response_text: Optional[str] = None
        async with self._agent.run_stream(
            user_prompt=user_prompt, message_history=message_history, event_stream_handler=event_stream_handler
        ) as response:
            async for text in response.stream_text():
                self._logger.info(f"Text: {text}")
                response_text = text
            # Message history is complete once the stream finishes
            all_messages: list[ModelMessage] = response.all_messages()
            self._save_history(request.state_id, all_messages)

        if response_text is None:
            return ContinuumAgentResponse(
                session_id=request.session_id,
                state_id=request.state_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No agent response received.",
            )

        return ContinuumAgentResponse(
            session_id=request.session_id, state_id=request.state_id, content_text=response_text
        )

    def shutdown(self):
        pass
