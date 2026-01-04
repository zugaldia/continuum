"""

Applications are a special kind of node that not only extends the regular queuing infrastructure, they also implement
the ContinuumExecutor execute_request function to kickstart/orchestrate the nodes they use to fulfill a request.

"""

from concurrent.futures import Future
from typing import Any, Optional, Callable
import time

from pydantic import BaseModel
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.apps.models import (
    ContinuumDictationRequest,
    ContinuumDictationResponse,
    ContinuumDictationStreamingResponse,
    DICTATION_STATUS_QUEUED,
    DICTATION_STATUS_TRANSCRIBED,
    DICTATION_STATUS_COMPLETED,
)
from continuum.constants import (
    CONTINUUM_NAMESPACE,
    ERROR_CODE_SUCCESS,
    ERROR_CODE_UNEXPECTED,
    PARAM_ASR_NODE,
    PARAM_ASR_NODE_DEFAULT,
    PARAM_LLM_NODE,
    PARAM_LLM_NODE_DEFAULT,
    PATH_ASR,
    PATH_LLM,
    QOS_DEPTH_DEFAULT,
    TOPIC_ASR_REQUEST,
    TOPIC_ASR_RESPONSE,
    TOPIC_ASR_STREAMING_RESPONSE,
    TOPIC_DICTATION_REQUEST,
    TOPIC_DICTATION_RESPONSE,
    TOPIC_DICTATION_STREAMING_RESPONSE,
    TOPIC_LLM_REQUEST,
    TOPIC_LLM_RESPONSE,
    TOPIC_LLM_STREAMING_RESPONSE,
)
from continuum.models import ContinuumExecutor
from continuum.utils import none_if_empty
from continuum_core.apps.base_app_node import BaseAppNode
from continuum_core.prompts.dictation import DICTATION_PROMPT, DEFAULT_CONTEXT
from continuum_interfaces.msg import (
    DictationResponse,
    DictationRequest,
    DictationStreamingResponse,
    AsrRequest,
    LlmRequest,
    AsrResponse,
    AsrStreamingResponse,
    LlmResponse,
    LlmStreamingResponse,
)


class SessionState(BaseModel):
    """Tracks the state and timing information for a dictation session."""

    request: ContinuumDictationRequest
    start_time: float
    asr_complete_time: Optional[float] = None


class DictationAppNode(BaseAppNode, ContinuumExecutor):
    _asr_publisher: Publisher[AsrRequest]
    _llm_publisher: Publisher[LlmRequest]
    _app_publisher: Publisher[DictationResponse]
    _app_streaming_publisher: Publisher[DictationStreamingResponse]
    _asr_node: str
    _llm_node: str
    _session_state: dict[str, SessionState]

    def __init__(self):
        super().__init__("dictation_app_node")
        self.set_node_info(
            name="Dictation app node",
            description="Dictation app that processes audio through ASR then through LLM for copy editing.",
        )

        self._executor = self
        self._session_state = {}
        self.get_logger().info(f"Dictation app node initialized with {self._asr_node}/{self._llm_node}.")

    #
    # Base node methods
    #

    def register_parameters(self) -> None:
        """Register the dictation app parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_ASR_NODE, PARAM_ASR_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            PARAM_LLM_NODE, PARAM_LLM_NODE_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        # Get ASR and LLM node names before registering publishers/subscribers
        self._asr_node = self._get_str_param(PARAM_ASR_NODE)
        self._llm_node = self._get_str_param(PARAM_LLM_NODE)

    def register_publishers(self) -> None:
        """Register the dictation response publishers."""
        self._app_publisher = self.create_publisher(DictationResponse, TOPIC_DICTATION_RESPONSE, QOS_DEPTH_DEFAULT)
        self._app_streaming_publisher = self.create_publisher(
            DictationStreamingResponse, TOPIC_DICTATION_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

        # Ability to issue ASR requests
        topic_asr_request = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_REQUEST}"
        self._asr_publisher = self.create_publisher(AsrRequest, topic_asr_request, QOS_DEPTH_DEFAULT)

        # Ability to issue LLM requests
        topic_llm_request = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{self._llm_node}/{TOPIC_LLM_REQUEST}"
        self._llm_publisher = self.create_publisher(LlmRequest, topic_llm_request, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the dictation request subscriber."""
        self.create_subscription(DictationRequest, TOPIC_DICTATION_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

        # Listen to ASR responses
        topic_asr_response = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_RESPONSE}"
        self.create_subscription(AsrResponse, topic_asr_response, self._on_asr_response, QOS_DEPTH_DEFAULT)

        # Listen to ASR updates
        topic_asr_updates = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/{self._asr_node}/{TOPIC_ASR_STREAMING_RESPONSE}"
        self.create_subscription(AsrStreamingResponse, topic_asr_updates, self._on_asr_updates, QOS_DEPTH_DEFAULT)

        # Listen to LLM responses
        topic_llm_response = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{self._llm_node}/{TOPIC_LLM_RESPONSE}"
        self.create_subscription(LlmResponse, topic_llm_response, self._on_llm_response, QOS_DEPTH_DEFAULT)

        # Listen to LLM updates
        topic_llm_updates = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/{self._llm_node}/{TOPIC_LLM_STREAMING_RESPONSE}"
        self.create_subscription(LlmStreamingResponse, topic_llm_updates, self._on_llm_updates, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: DictationRequest) -> None:
        """Handle incoming dictation requests."""
        self._log_message_redacted(msg)
        sdk_request = ContinuumDictationRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    async def execute_request(
        self,
        request: ContinuumDictationRequest,
        streaming_callback: Optional[Callable[[ContinuumDictationStreamingResponse], None]] = None,
    ) -> ContinuumDictationResponse:
        self._logger.info(f"Starting dictation request for session_id: {request.session_id}")
        self.add_active_session(request.session_id)
        self._session_state[request.session_id] = SessionState(request=request, start_time=time.time())
        self._asr_publisher.publish(
            AsrRequest(
                session_id=request.session_id,
                audio_data=request.audio_data,
                format=request.format,
                channels=request.channels,
                sample_rate=request.sample_rate,
                sample_width=request.sample_width,
                language=request.language,
            )
        )

        # Publish (queued) update to consumers
        return ContinuumDictationResponse(session_id=request.session_id, status=DICTATION_STATUS_QUEUED)

    def _on_asr_response(self, msg: AsrResponse) -> None:
        """Handle incoming ASR responses."""
        # Only process responses for sessions we initiated
        if not self.has_active_session(msg.session_id):
            return

        # Retrieve the session state
        session_state = self._session_state.get(msg.session_id)
        if session_state is None:
            self.get_logger().error(f"No stored session state found for session_id: {msg.session_id}")
            return

        # Record ASR completion time
        session_state.asr_complete_time = time.time()

        # Check if the ASR response contains an error
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"ASR error received: {msg.error_message}")
            self.publish_app_response(
                ContinuumDictationResponse(
                    session_id=msg.session_id,
                    error_code=msg.error_code,
                    error_message=msg.error_message,
                )
            )
            self._cleanup_session(msg.session_id)
            return

        transcription = none_if_empty(msg.transcription)
        if transcription is None:
            error_message = f"ASR produced an empty transcription ({msg.transcription})."
            self.get_logger().error(error_message)
            self.publish_app_response(
                ContinuumDictationResponse(
                    session_id=msg.session_id,
                    error_code=ERROR_CODE_UNEXPECTED,
                    error_message=error_message,
                )
            )
            self._cleanup_session(msg.session_id)
            return

        # Not only this provides an intermediate result to the consumer to show actual progress to the user,
        # if the LLM request fails for some reason, this text could be used as fallback.
        self.publish_app_response(
            ContinuumDictationResponse(
                session_id=msg.session_id, content_text=transcription, status=DICTATION_STATUS_TRANSCRIBED
            )
        )

        # Request final pass by the LLM, using context from the original request
        context = none_if_empty(session_state.request.context) or DEFAULT_CONTEXT
        content_text = DICTATION_PROMPT.format(CONTEXT=context, INPUT=transcription.strip())
        llm_request = LlmRequest(session_id=msg.session_id, content_text=content_text)
        self._llm_publisher.publish(llm_request)

    def _on_asr_updates(self, msg: AsrStreamingResponse) -> None:
        """Handle incoming ASR streaming responses."""
        # Only process updates for sessions we initiated
        if not self.has_active_session(msg.session_id):
            return
        self.publish_app_streaming_response(ContinuumDictationStreamingResponse(session_id=msg.session_id))

    def _on_llm_response(self, msg: LlmResponse) -> None:
        """Handle incoming LLM responses."""
        self.get_logger().info(f"LLM response received: {msg}")

        # Only process responses for sessions we initiated
        if not self.has_active_session(msg.session_id):
            return

        # Retrieve the session state to calculate time stats
        session_state = self._session_state.get(msg.session_id)
        if session_state is None:
            self.get_logger().error(f"No stored session state found for session_id: {msg.session_id}")
            return

        # Total duration
        end_time = time.time()
        total_duration = end_time - session_state.start_time

        # ASR duration
        asr_duration = 0.0
        if session_state.asr_complete_time is not None:
            asr_duration = session_state.asr_complete_time - session_state.start_time

        # LLM duration
        llm_duration = 0.0
        if session_state.asr_complete_time is not None:
            llm_duration = end_time - session_state.asr_complete_time

        # Publish the final response
        if msg.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"LLM error received: {msg.error_message}")
            self.publish_app_response(
                ContinuumDictationResponse(
                    session_id=msg.session_id,
                    error_code=msg.error_code,
                    error_message=msg.error_message,
                    asr_node=self._asr_node,
                    llm_node=self._llm_node,
                    asr_duration_seconds=asr_duration,
                    llm_duration_seconds=llm_duration,
                    total_duration_seconds=total_duration,
                )
            )
        else:
            self.publish_app_response(
                ContinuumDictationResponse(
                    session_id=msg.session_id,
                    content_text=msg.content_text,
                    status=DICTATION_STATUS_COMPLETED,
                    asr_node=self._asr_node,
                    llm_node=self._llm_node,
                    asr_duration_seconds=asr_duration,
                    llm_duration_seconds=llm_duration,
                    total_duration_seconds=total_duration,
                )
            )

        # Clean up session state
        self._cleanup_session(msg.session_id)

    def _on_llm_updates(self, msg: LlmStreamingResponse) -> None:
        """Handle incoming LLM streaming responses."""
        # Only process updates for sessions we initiated
        if not self.has_active_session(msg.session_id):
            return
        self.publish_app_streaming_response(ContinuumDictationStreamingResponse(session_id=msg.session_id))

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumDictationResponse], sdk_request: ContinuumDictationRequest) -> None:
        """Handle the result of a dictation request."""
        try:
            sdk_response = future.result()
            self.publish_app_response(sdk_response)
        except Exception as e:
            self.get_logger().error(f"Error processing request: {e}")
            self.publish_app_response(
                ContinuumDictationResponse(
                    session_id=sdk_request.session_id,
                    error_code=ERROR_CODE_UNEXPECTED,
                    error_message=str(e),
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(
        self,
        streaming_response: ContinuumDictationStreamingResponse,
        sdk_request: ContinuumDictationRequest,
    ) -> None:
        """Handle a streaming result from a dictation request."""
        self.publish_app_streaming_response(streaming_response)

    #
    # Custom dictation app methods
    #

    def _cleanup_session(self, session_id: str) -> None:
        """Clean up session state and tracking."""
        self._session_state.pop(session_id, None)
        self.remove_active_session(session_id)

    def publish_app_response(self, sdk_response: ContinuumDictationResponse) -> None:
        response = DictationResponse()
        set_message_fields(response, sdk_response.model_dump())
        self.get_logger().info(f"Dictation response: {sdk_response}")
        self._app_publisher.publish(response)

    def publish_app_streaming_response(self, sdk_streaming_response: ContinuumDictationStreamingResponse) -> None:
        response = DictationStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self.get_logger().debug(f"Dictation streaming response: {sdk_streaming_response}")
        self._app_streaming_publisher.publish(response)


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            dictation_app_node = DictationAppNode()
            rclpy.spin(dictation_app_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
