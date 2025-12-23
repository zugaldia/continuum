"""Base class for LLM nodes."""

from abc import ABC
from concurrent.futures import Future

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.constants import (
    TOPIC_LLM_RESPONSE,
    TOPIC_LLM_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_LLM_REQUEST,
    ERROR_CODE_UNEXPECTED,
)
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmResponse, ContinuumLlmStreamingResponse
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import LlmResponse, LlmRequest, LlmStreamingResponse


class BaseLlmNode(QueueNode, ABC):
    """Base class for all LLM nodes in Continuum."""

    _llm_publisher: Publisher[LlmResponse]
    _llm_streaming_publisher: Publisher[LlmStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_publishers(self) -> None:
        """Register the LLM response publishers."""
        self._llm_publisher = self.create_publisher(LlmResponse, TOPIC_LLM_RESPONSE, QOS_DEPTH_DEFAULT)
        self._llm_streaming_publisher = self.create_publisher(
            LlmStreamingResponse, TOPIC_LLM_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the LLM request subscriber."""
        self.create_subscription(LlmRequest, TOPIC_LLM_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: LlmRequest) -> None:
        """Handle incoming LLM requests."""
        self.get_logger().info(f"LLM request received with session_id: {msg.session_id}")
        sdk_request = ContinuumLlmRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumLlmResponse], sdk_request: ContinuumLlmRequest) -> None:
        """Handle the result of an LLM request."""
        try:
            sdk_response = future.result()
            self.publish_llm_response(sdk_response)
        except Exception as e:
            self.get_logger().error(f"Error processing request: {e}")
            self.publish_llm_response(
                ContinuumLlmResponse(
                    session_id=sdk_request.session_id, error_code=ERROR_CODE_UNEXPECTED, error_message=str(e)
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(
        self, streaming_response: ContinuumLlmStreamingResponse, sdk_request: ContinuumLlmRequest
    ) -> None:
        """Handle a streaming result from an LLM request."""
        self.publish_llm_streaming_response(streaming_response)

    #
    # Custom publishing methods
    #

    def publish_llm_response(self, sdk_response: ContinuumLlmResponse) -> None:
        response = LlmResponse()
        set_message_fields(response, sdk_response.model_dump())
        self.get_logger().info(f"LLM response: {sdk_response}")
        self._llm_publisher.publish(response)

    def publish_llm_streaming_response(self, sdk_streaming_response: ContinuumLlmStreamingResponse) -> None:
        response = LlmStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self.get_logger().info(f"LLM streaming response: {sdk_streaming_response}")
        self._llm_streaming_publisher.publish(response)
