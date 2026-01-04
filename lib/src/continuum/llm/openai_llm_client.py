import logging
from typing import Optional, Callable

from openai import OpenAI, Omit
from openai.types import Reasoning
from openai.types.responses import (
    ResponseCompletedEvent,
    ResponseContentPartAddedEvent,
    ResponseContentPartDoneEvent,
    ResponseCreatedEvent,
    ResponseInProgressEvent,
    ResponseOutputItemAddedEvent,
    ResponseOutputItemDoneEvent,
    ResponseTextDeltaEvent,
    ResponseTextDoneEvent,
    EasyInputMessageParam,
)

from continuum.constants import ERROR_CODE_UNEXPECTED, DEFAULT_MODEL_NAME_OPENAI
from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import (
    ContinuumLlmResponse,
    ContinuumLlmRequest,
    ContinuumLlmStreamingResponse,
    OpenAiLlmOptions,
)
from continuum.utils import none_if_empty, is_empty


class OpenAiLlmClient(ContinuumLlmClient):
    """OpenAI LLM client."""

    def __init__(self, options: OpenAiLlmOptions = OpenAiLlmOptions()) -> None:
        """Initialize the OpenAI LLM client."""
        self._logger = logging.getLogger(__name__)
        self._options = options
        self._client = OpenAI(api_key=none_if_empty(options.api_key), base_url=none_if_empty(options.base_url))
        self._logger.info("OpenAI LLM client initialized.")

    async def execute_request(
        self,
        request: ContinuumLlmRequest,
        streaming_callback: Optional[Callable[[ContinuumLlmStreamingResponse], None]] = None,
    ) -> ContinuumLlmResponse:
        """Execute LLM request and return response with streaming support."""
        model_name = none_if_empty(self._options.model_name) or DEFAULT_MODEL_NAME_OPENAI
        previous_response_id = (
            Omit()
            # Do not add a response ID for custom/local endpoints (or if none is provided)
            if (none_if_empty(self._options.base_url) is not None or none_if_empty(request.state_id) is None)
            else request.state_id
        )

        self._logger.info(
            f"New LLM request ({model_name}) for session_id (state={previous_response_id}): {request.session_id}"
        )

        instructions = Omit() if is_empty(request.system_prompt) else request.system_prompt
        events = self._client.responses.create(
            model=model_name,
            instructions=instructions,
            previous_response_id=previous_response_id,
            reasoning=Reasoning(effort="none"),
            input=[EasyInputMessageParam(role="user", content=request.content_text)],
            stream=True,
        )

        # Currently, we essentially ignore all events except the delta and the completed response,
        # but we are ready to add more details if needed.
        completed_event: Optional[ResponseCompletedEvent] = None
        for event in events:
            # Emitted when the model response is complete.
            if isinstance(event, ResponseCompletedEvent):
                completed_event = event
            # Emitted when a new content part is added.
            elif isinstance(event, ResponseContentPartAddedEvent):
                self._logger.info("-> ResponseContentPartAddedEvent")
            # Emitted when a content part is done.
            elif isinstance(event, ResponseContentPartDoneEvent):
                self._logger.info("-> ResponseContentPartDoneEvent")
            # An event that is emitted when a response is created.
            elif isinstance(event, ResponseCreatedEvent):
                self._logger.info("-> ResponseCreatedEvent")
            # Emitted when the response is in progress.
            elif isinstance(event, ResponseInProgressEvent):
                self._logger.info("-> ResponseInProgressEvent")
            # Emitted when a new output item is added.
            elif isinstance(event, ResponseOutputItemAddedEvent):
                self._logger.info("-> ResponseOutputItemAddedEvent")
            # Emitted when an output item is marked done.
            elif isinstance(event, ResponseOutputItemDoneEvent):
                self._logger.info("-> ResponseOutputItemDoneEvent")
            # Emitted when there is an additional text delta.
            elif isinstance(event, ResponseTextDeltaEvent):
                if streaming_callback:
                    streaming_callback(
                        ContinuumLlmStreamingResponse(session_id=request.session_id, content_text=event.delta)
                    )
            # Emitted when text content is finalized.
            elif isinstance(event, ResponseTextDoneEvent):
                self._logger.info("-> ResponseTextDoneEvent")
            else:
                self._logger.warning(f"Unknown event type: {event}")

        if completed_event:
            content_text = completed_event.response.output[0].content[0].text
            done_reason = completed_event.response.status
            response_id = completed_event.response.id
            return ContinuumLlmResponse(
                session_id=request.session_id,
                state_id=response_id,
                content_text=content_text,
                done_reason=done_reason,
            )
        else:
            return ContinuumLlmResponse(
                session_id=request.session_id,
                error_code=ERROR_CODE_UNEXPECTED,
                error_message="No completed event received.",
            )

    def shutdown(self) -> None:
        """Shutdown the LLM client and clean up resources."""
        self._logger.info("OpenAI LLM client shutting down.")
