import logging
from typing import Optional, Callable, List

from google import genai
from google.genai._interactions import Omit
from google.genai._interactions.types import (
    InteractionEvent,
    InteractionStatusUpdate,
    ContentStart,
    ContentStop,
    ContentDelta,
    GenerationConfigParam,
    ErrorEvent,
)

from continuum.constants import ERROR_CODE_UNEXPECTED, DEFAULT_MODEL_NAME_GOOGLE
from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import (
    ContinuumLlmResponse,
    ContinuumLlmRequest,
    ContinuumLlmStreamingResponse,
    GoogleLlmOptions,
)
from continuum.utils import none_if_empty, is_empty


class GoogleLlmClient(ContinuumLlmClient):
    """Google LLM client."""

    def __init__(
        self,
        options: GoogleLlmOptions = GoogleLlmOptions(),
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> None:
        """Initialize the Google LLM client."""
        super().__init__(streaming_callback)
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._client = genai.Client(api_key=none_if_empty(options.api_key))
        self._logger.info("Google LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response with streaming support."""
        model_name = none_if_empty(self._options.model_name) or DEFAULT_MODEL_NAME_GOOGLE
        previous_interaction_id = none_if_empty(request.state_id) or Omit()
        stateful = none_if_empty(request.state_id) is not None

        # Do not include the system instructions if a previous interaction ID is set?
        system_instruction = Omit() if is_empty(request.system_prompt) else request.system_prompt
        self._logger.info(f"New LLM request ({model_name}) for session_id (stateful={stateful}): {request.session_id}")

        stream = self._client.interactions.create(
            input=request.content_text,
            model=model_name,
            generation_config=GenerationConfigParam(thinking_level="minimal"),
            previous_interaction_id=previous_interaction_id,
            stream=True,
            system_instruction=system_instruction,
        )

        deltas: List[ContentDelta] = []
        final_event: Optional[InteractionEvent] = None
        for chunk in stream:
            if isinstance(chunk, InteractionEvent):
                self._logger.info(f"Interaction event: {chunk.interaction.status}")
                if chunk.event_type == "interaction.complete":
                    final_event = chunk
            elif isinstance(chunk, InteractionStatusUpdate):
                self._logger.info(f"Interaction status update: {chunk.status}")
            elif isinstance(chunk, ErrorEvent):
                error_message = f"Error event ({chunk.error.code}): {chunk.error.message}"
                self._logger.error(error_message)
                return ContinuumLlmResponse(
                    session_id=request.session_id,
                    error_code=ERROR_CODE_UNEXPECTED,
                    error_message=error_message,
                )
            elif isinstance(chunk, ContentStart):
                pass
            elif isinstance(chunk, ContentStop):
                pass
            elif isinstance(chunk, ContentDelta):
                if chunk.delta.type == "text":  # Could also be "thought"
                    deltas.append(chunk)
                    if self.streaming_callback:
                        self.streaming_callback(
                            ContinuumLlmStreamingResponse(session_id=request.session_id, content_text=chunk.delta.text)
                        )
            else:
                self._logger.warning(f"Unknown interaction event: {type(chunk)}")

        if not deltas or not final_event:
            self._logger.error(f"No outputs received for session_id: {request.session_id}")
            return ContinuumLlmResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No outputs received from Google LLM.",
            )

        content_text = "".join([chunk.delta.text for chunk in deltas])
        state_id = final_event.interaction.id
        done_reason = final_event.interaction.status
        response = ContinuumLlmResponse(
            session_id=request.session_id,
            state_id=state_id,
            content_text=content_text,
            done_reason=done_reason,
        )

        self._logger.info(f"LLM completed for session_id: {request.session_id}")
        return response

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("Google LLM client shutting down.")
