import logging
from typing import Sequence, AsyncIterator, Optional, Callable

from ollama import AsyncClient, Message, ChatResponse

from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import ContinuumLlmResponse, ContinuumLlmRequest, ContinuumLlmStreamingResponse


class OllamaLlmClient(ContinuumLlmClient):
    """Ollama LLM client for testing purposes."""

    def __init__(self) -> None:
        """Initialize the fake LLM client."""
        self._logger = logging.getLogger(__name__)
        self._client = AsyncClient(host="http://orin:11434")
        self._logger.info("Ollama LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response with streaming support."""
        self._logger.info(f"Starting LLM request for session_id: {request.session_id}")

        model: str = "mistral-small"
        inputs: Sequence[Message] = [Message(role="user", content=request.content_text)]
        stream: bool = True
        think: bool = False
        results: ChatResponse | AsyncIterator[ChatResponse] = await self._client.chat(
            model=model, messages=inputs, stream=stream, think=think
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

        content_text = "".join([output.message.content for output in outputs])
        done_reason = outputs[-1].done_reason if outputs else None
        response = ContinuumLlmResponse(
            session_id=request.session_id,
            content_text=content_text,
            done_reason=done_reason,
        )

        self._logger.info(f"LLM completed for session_id: {request.session_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("Ollama LLM client shutting down.")
