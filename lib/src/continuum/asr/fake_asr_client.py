import asyncio
import logging
import random

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.models import ContinuumAsrResponse, ContinuumAsrRequest


class FakeAsrClient(ContinuumAsrClient):
    """Fake ASR (Automatic Speech Recognition) client for testing purposes."""

    def __init__(self):
        """Initialize the fake ASR client."""
        self._logger = logging.getLogger(__name__)
        self._logger.info("Fake ASR client initialized.")

    async def execute_request(self, request: ContinuumAsrRequest) -> ContinuumAsrResponse:
        """Transcribe audio data to text."""
        self._logger.info(f"Starting transcription for session_id: {request.session_id}")

        await asyncio.sleep(5)  # Simulate ASR latency

        # Chaos Monkey: Randomly throw an exception 25% of the time
        if random.random() < 0.25:
            self._logger.error(f"Fake ASR error for session_id: {request.session_id}")
            raise Exception("Fake ASR error")

        response = ContinuumAsrResponse(session_id=request.session_id, transcription="This is a fake transcription.")
        self._logger.info(f"Transcription completed for session_id: {request.session_id}")
        return response

    def shutdown(self):
        """Shutdown the ASR client and clean up resources."""
        self._logger.info("Fake ASR client shutting down.")
