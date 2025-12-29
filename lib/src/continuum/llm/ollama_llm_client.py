import logging
from typing import Sequence, AsyncIterator, Optional, Callable

from ollama import AsyncClient, Message, ChatResponse

from continuum.constants import ERROR_CODE_UNEXPECTED, DEFAULT_MODEL_NAME_OLLAMA
from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import (
    ContinuumLlmResponse,
    ContinuumLlmRequest,
    ContinuumLlmStreamingResponse,
    OllamaLlmOptions,
)
from continuum.utils import none_if_empty, generate_unique_id


class OllamaLlmClient(ContinuumLlmClient):
    """Ollama LLM client for testing purposes."""

    def __init__(self, options: OllamaLlmOptions = OllamaLlmOptions()) -> None:
        """Initialize the fake LLM client."""
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._client = AsyncClient(host=options.host)
        self._logger.info("Ollama LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response with streaming support."""
        model_name = none_if_empty(self._options.model_name) or DEFAULT_MODEL_NAME_OLLAMA

        self._logger.info(f"Starting LLM request ({model_name}) for session_id: {request.session_id}")
        inputs: Sequence[Message] = [Message(role="user", content=request.content_text)]
        results: ChatResponse | AsyncIterator[ChatResponse] = await self._client.chat(
            model=model_name, messages=inputs, stream=True, think=False
        )

        outputs: list[ChatResponse] = []
        async for result in results:
            outputs.append(result)
            if result.done:
                self._logger.info(f"LLM is done: {result}")
            elif streaming_callback:
                streaming_callback(
                    ContinuumLlmStreamingResponse(session_id=request.session_id, content_text=result.message.content)
                )

        if not outputs:
            self._logger.error(f"No outputs received for session_id: {request.session_id}")
            return ContinuumLlmResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No outputs received from Ollama LLM.",
            )

        content_text = "".join([output.message.content for output in outputs])
        done_reason = outputs[-1].done_reason

        # Generate a state_id for conversation tracking
        # Note: Ollama doesn't have a built-in conversation state like OpenAI/Google,
        # so we generate a UUID to maintain consistency with other providers
        state_id = generate_unique_id()

        response = ContinuumLlmResponse(
            session_id=request.session_id,
            state_id=state_id,
            content_text=content_text,
            done_reason=done_reason,
        )

        self._logger.info(f"LLM completed for session_id: {request.session_id}, state_id: {state_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("Ollama LLM client shutting down.")
