import asyncio
import logging
import random
from typing import Optional, Callable

from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import (
    ContinuumLlmResponse,
    ContinuumLlmRequest,
    ContinuumLlmStreamingResponse,
    FakeLlmOptions,
)

# Fake LLM text used for testing
FAKE_RESPONSE = "You are absolutely right!"


class FakeLlmClient(ContinuumLlmClient):
    """Fake LLM client for testing purposes."""

    def __init__(
        self,
        options: FakeLlmOptions = FakeLlmOptions(),
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the fake LLM client."""
        super().__init__(streaming_callback)
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._logger.info("Fake LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response."""
        self._logger.info(f"Starting LLM request for session_id: {request.session_id}")

        # Chaos Monkey: Randomly throw an exception
        if random.random() < self._options.error_rate:
            self._logger.error(f"Fake LLM error for session_id: {request.session_id}")
            raise Exception("Fake LLM error, courtesy of the Chaos Monkey.")

        # Stream words one at a time if the streaming callback is provided
        words = FAKE_RESPONSE.split()
        for i, word in enumerate(words):
            await asyncio.sleep(self._options.streaming_delay_seconds)  # Simulate token-by-token processing delay
            if self.streaming_callback:
                self._logger.debug(f"Intermediate result for session_id: {request.session_id}: {word}")
                self.streaming_callback(ContinuumLlmStreamingResponse(session_id=request.session_id, content_text=word))

        # Return final response
        response = ContinuumLlmResponse(session_id=request.session_id, content_text=FAKE_RESPONSE, done_reason="tired")

        self._logger.info(f"LLM completed for session_id: {request.session_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("Fake LLM client shutting down.")
