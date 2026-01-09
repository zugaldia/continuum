"""Base class for VAD (Voice Activity Detection) nodes."""

from abc import ABC
from concurrent.futures import Future

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.vad.models import ContinuumVadRequest, ContinuumVadResponse, ContinuumVadStreamingResponse
from continuum.constants import (
    TOPIC_VAD_RESPONSE,
    TOPIC_VAD_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_VAD_REQUEST,
    ERROR_CODE_UNEXPECTED,
    ERROR_CODE_SUCCESS,
)
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import VadResponse, VadRequest, VadStreamingResponse


class BaseVadNode(QueueNode, ABC):
    """Base class for all VAD nodes in Continuum."""

    _vad_publisher: Publisher[VadResponse]
    _vad_streaming_publisher: Publisher[VadStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_publishers(self) -> None:
        """Register the VAD response publishers."""
        self._vad_publisher = self.create_publisher(VadResponse, TOPIC_VAD_RESPONSE, QOS_DEPTH_DEFAULT)
        self._vad_streaming_publisher = self.create_publisher(
            VadStreamingResponse, TOPIC_VAD_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the VAD request subscriber."""
        self.create_subscription(VadRequest, TOPIC_VAD_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: VadRequest) -> None:
        """Handle incoming VAD requests."""
        if self.debug_mode:
            self.get_logger().info(f"VAD request received with session_id: {msg.session_id}")
        sdk_request = ContinuumVadRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumVadResponse], sdk_request: ContinuumVadRequest) -> None:
        """Handle the result of a VAD request."""
        try:
            sdk_response = future.result()
            self.publish_vad_response(sdk_response)
        except Exception as e:
            self.publish_vad_response(
                ContinuumVadResponse(
                    session_id=sdk_request.session_id,
                    error_code=ERROR_CODE_UNEXPECTED,
                    error_message=str(e),
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(self, streaming_response: ContinuumVadStreamingResponse) -> None:
        """Handle a streaming result from a VAD request."""
        self.publish_vad_streaming_response(streaming_response)

    #
    # Custom publishing methods
    #

    def publish_vad_response(self, sdk_response: ContinuumVadResponse) -> None:
        response = VadResponse()
        set_message_fields(response, sdk_response.model_dump())
        if sdk_response.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"VAD response error: {sdk_response}")
        elif self.debug_mode:
            self._log_message_redacted(response)
        self._vad_publisher.publish(response)

    def publish_vad_streaming_response(self, sdk_streaming_response: ContinuumVadStreamingResponse) -> None:
        self.get_logger().debug(f"VAD streaming response: {sdk_streaming_response}")
        response = VadStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self._vad_streaming_publisher.publish(response)
