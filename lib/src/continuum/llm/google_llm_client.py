import logging
from typing import Optional, Callable, Iterator, List

from google import genai
from google.genai import types

from continuum.constants import ERROR_CODE_UNEXPECTED
from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import (
    ContinuumLlmResponse,
    ContinuumLlmRequest,
    ContinuumLlmStreamingResponse,
    GoogleLlmOptions,
)
from continuum.utils import none_if_empty


class GoogleLlmClient(ContinuumLlmClient):
    """Google LLM client."""

    def __init__(self, options: GoogleLlmOptions = GoogleLlmOptions()) -> None:
        """Initialize the Google LLM client."""
        self._logger = logging.getLogger(__name__)
        self._client = genai.Client(api_key=none_if_empty(options.api_key))
        self._logger.info("Google LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response with streaming support."""
        self._logger.info(f"Starting LLM request for session_id: {request.session_id}")

        events: Iterator[types.GenerateContentResponse] = self._client.models.generate_content_stream(
            model="gemini-3-flash-preview",
            contents=[request.content_text],
            config=types.GenerateContentConfig(thinking_config=types.ThinkingConfig(thinking_level="minimal")),
        )

        outputs: List[types.GenerateContentResponse] = []
        for event in events:
            outputs.append(event)
            if self._get_finish_reason(event):
                self._logger.info("LLM is done.")
            elif streaming_callback:
                streaming_callback(
                    ContinuumLlmStreamingResponse(session_id=request.session_id, content_text=self._get_text(event))
                )

        if not outputs:
            self._logger.error(f"No outputs received for session_id: {request.session_id}")
            return ContinuumLlmResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No outputs received from Google LLM.",
            )

        content_text = "".join([self._get_text(output) for output in outputs])
        done_reason = self._get_finish_reason(outputs[-1])
        response = ContinuumLlmResponse(
            session_id=request.session_id,
            content_text=content_text,
            done_reason=done_reason,
        )

        self._logger.info(f"LLM completed for session_id: {request.session_id}")
        return response

    def _get_text(self, event: types.GenerateContentResponse) -> str:
        try:
            # This is an Optional[str]
            return event.candidates[0].content.parts[0].text or ""
        except Exception as e:
            self._logger.warning(f"Failed to extract text from event: {e}")
            return ""

    def _get_finish_reason(self, event: types.GenerateContentResponse) -> str:
        try:
            finish_reason = event.candidates[0].finish_reason
            return str(finish_reason) if finish_reason else ""
        except Exception as e:
            self._logger.warning(f"Failed to extract finish_reason from event: {e}")
            return ""

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("Google LLM client shutting down.")
