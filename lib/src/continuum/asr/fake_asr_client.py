import asyncio
import logging
import random
from typing import Optional, Callable

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.models import (
    ContinuumAsrResponse,
    ContinuumAsrRequest,
    ContinuumAsrStreamingResponse,
    FakeAsrOptions,
)

# Fake transcription text used for testing
FAKE_TRANSCRIPTION = "Testing one two three is this thing on"


class FakeAsrClient(ContinuumAsrClient):
    """Fake ASR (Automatic Speech Recognition) client for testing purposes."""

    def __init__(
        self,
        options: FakeAsrOptions = FakeAsrOptions(),
        streaming_callback: Optional[Callable[[ContinuumAsrStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the fake ASR client."""
        super().__init__(streaming_callback)
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._logger.info("Fake ASR client initialized.")

    async def execute_request(
        self,
        request: ContinuumAsrRequest,
    ) -> ContinuumAsrResponse:
        """Transcribe audio data to text."""
        self._logger.info(f"Starting transcription for session_id: {request.session_id}")

        # Chaos Monkey: Randomly throw an exception
        if random.random() < self._options.error_rate:
            self._logger.error(f"Fake ASR error for session_id: {request.session_id}")
            raise Exception("Fake ASR error, courtesy of the Chaos Monkey.")

        # Stream words one at a time if the streaming callback is provided
        words = FAKE_TRANSCRIPTION.split()
        for i, word in enumerate(words):
            await asyncio.sleep(self._options.streaming_delay_seconds)  # Simulate word-by-word processing delay
            if self.streaming_callback:
                self._logger.debug(f"Intermediate result for session_id: {request.session_id}: {word}")
                self.streaming_callback(
                    ContinuumAsrStreamingResponse(session_id=request.session_id, transcription=word)
                )

        # Return final response
        response = ContinuumAsrResponse(session_id=request.session_id, transcription=FAKE_TRANSCRIPTION)

        self._logger.info(f"Transcription completed for session_id: {request.session_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the ASR client and clean up resources."""
        self._logger.info("Fake ASR client shutting down.")
