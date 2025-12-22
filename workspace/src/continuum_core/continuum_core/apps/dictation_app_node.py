"""

Applications are a special kind of node that not only extends the regular queuing infrastructure, they also implement
the ContinuumClient execute_request function to kickstart/orchestrate the nodes they utilize to fulfill a request.

"""

from concurrent.futures import Future
from typing import Any, Optional, Callable

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.apps.models import (
    ContinuumDictationRequest,
    ContinuumDictationResponse,
    ContinuumDictationStreamingResponse, DICTATION_STATUS_QUEUED, DICTATION_STATUS_TRANSCRIBED,
    DICTATION_STATUS_COMPLETED,
)
from continuum.constants import (
    TOPIC_DICTATION_RESPONSE,
    TOPIC_DICTATION_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_DICTATION_REQUEST,
    ERROR_CODE_UNEXPECTED, CONTINUUM_NAMESPACE, TOPIC_ASR_REQUEST, TOPIC_LLM_REQUEST, TOPIC_ASR_RESPONSE,
    TOPIC_ASR_STREAMING_RESPONSE, TOPIC_LLM_RESPONSE, TOPIC_LLM_STREAMING_RESPONSE, PATH_ASR, PATH_LLM,
)
from continuum.models import ContinuumClient
from continuum_core.apps.base_app_node import BaseAppNode
from continuum_interfaces.msg import (
    DictationResponse,
    DictationRequest,
    DictationStreamingResponse, AsrRequest, LlmRequest, AsrResponse, AsrStreamingResponse, LlmResponse,
    LlmStreamingResponse,
)


class DictationAppNode(BaseAppNode, ContinuumClient):
    _asr_publisher: Publisher[AsrRequest]
    _llm_publisher: Publisher[LlmRequest]
    _app_publisher: Publisher[DictationResponse]
    _app_streaming_publisher: Publisher[DictationStreamingResponse]

    def __init__(self):
        super().__init__("dictation_app_node")
        self.set_node_info(
            name="Dictation app node",
            description="Dictation app that processes audio through ASR then through LLM for copy editing.",
        )

        self._client = self
        self.get_logger().info("Dictation app node initialized.")

    #
    # Base node methods
    #

    def register_publishers(self) -> None:
        """Register the dictation response publishers."""
        self._app_publisher = self.create_publisher(DictationResponse, TOPIC_DICTATION_RESPONSE, QOS_DEPTH_DEFAULT)
        self._app_streaming_publisher = self.create_publisher(
            DictationStreamingResponse, TOPIC_DICTATION_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

        # Ability to issue ASR requests
        topic1 = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/faster_whisper/{TOPIC_ASR_REQUEST}"
        self._asr_publisher = self.create_publisher(AsrRequest, topic1, QOS_DEPTH_DEFAULT)

        # Ability to issue LLM requests
        topic2 = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/ollama/{TOPIC_LLM_REQUEST}"
        self._llm_publisher = self.create_publisher(LlmRequest, topic2, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the dictation request subscriber."""
        self.create_subscription(DictationRequest, TOPIC_DICTATION_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

        # Listen to ASR responses
        topic_asr_response = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/faster_whisper/{TOPIC_ASR_RESPONSE}"
        self.create_subscription(AsrResponse, topic_asr_response, self._on_asr_response, QOS_DEPTH_DEFAULT)

        # Listen to ASR updates
        topic_asr_updates = f"/{CONTINUUM_NAMESPACE}/{PATH_ASR}/faster_whisper/{TOPIC_ASR_STREAMING_RESPONSE}"
        self.create_subscription(AsrStreamingResponse, topic_asr_updates, self._on_asr_updates, QOS_DEPTH_DEFAULT)

        # Listen to LLM responses
        topic_llm_response = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/ollama/{TOPIC_LLM_RESPONSE}"
        self.create_subscription(LlmResponse, topic_llm_response, self._on_llm_response, QOS_DEPTH_DEFAULT)

        # Listen to LLM updates
        topic_llm_updates = f"/{CONTINUUM_NAMESPACE}/{PATH_LLM}/ollama/{TOPIC_LLM_STREAMING_RESPONSE}"
        self.create_subscription(LlmStreamingResponse, topic_llm_updates, self._on_llm_updates, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: DictationRequest) -> None:
        """Handle incoming dictation requests."""
        self.get_logger().info(f"Dictation request received with session_id: {msg.session_id}")
        sdk_request = ContinuumDictationRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    async def execute_request(
            self, request: ContinuumDictationRequest, streaming_callback: Optional[
                Callable[[ContinuumDictationStreamingResponse], None]] = None) -> ContinuumDictationResponse:
        self._logger.info(f"Starting dictation request for session_id: {request.session_id}")
        asr_request = AsrRequest(session_id=request.session_id, audio_path=request.audio_path)
        self._asr_publisher.publish(asr_request)
        return ContinuumDictationResponse(session_id=request.session_id, status=DICTATION_STATUS_QUEUED)

    def _on_asr_response(self, msg: AsrResponse) -> None:
        """Handle incoming ASR responses."""
        self.get_logger().info(f"ASR response received: {msg}")

        # Not only this provides an intermediate result to the consumer to show actual progress to the user,
        # if the LLM request fails for some reason, this text could be used as fallback.
        self.publish_app_response(ContinuumDictationResponse(
            session_id=msg.session_id, content_text=msg.transcription, status=DICTATION_STATUS_TRANSCRIBED))

        # Request final pass by the LLM
        content_text = f"Copyedit the following text: {msg.transcription}"
        llm_request = LlmRequest(session_id=msg.session_id, content_text=content_text)
        self._llm_publisher.publish(llm_request)

    def _on_asr_updates(self, msg: AsrStreamingResponse) -> None:
        """Handle incoming ASR streaming responses."""
        self.publish_app_streaming_response(ContinuumDictationStreamingResponse(session_id=msg.session_id))

    def _on_llm_response(self, msg: LlmResponse) -> None:
        """Handle incoming LLM responses."""
        self.get_logger().info(f"LLM response received: {msg}")
        self.publish_app_response(ContinuumDictationResponse(
            session_id=msg.session_id, content_text=msg.content_text, status=DICTATION_STATUS_COMPLETED))

    def _on_llm_updates(self, msg: LlmStreamingResponse) -> None:
        """Handle incoming LLM streaming responses."""
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

    def publish_app_response(self, sdk_response: ContinuumDictationResponse) -> None:
        response = DictationResponse()
        set_message_fields(response, sdk_response.model_dump())
        self.get_logger().info(f"Dictation response: {sdk_response}")
        self._app_publisher.publish(response)

    def publish_app_streaming_response(self, sdk_streaming_response: ContinuumDictationStreamingResponse) -> None:
        response = DictationStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self.get_logger().info(f"Dictation streaming response: {sdk_streaming_response}")
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
