"""Base class for ASR (Automatic Speech Recognition) nodes."""

from abc import ABC
from concurrent.futures import Future

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrResponse, ContinuumAsrStreamingResponse
from continuum.constants import (
    TOPIC_ASR_RESPONSE,
    TOPIC_ASR_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_ASR_REQUEST,
    ERROR_CODE_UNEXPECTED,
    ERROR_CODE_SUCCESS,
    PARAM_ASR_MODEL_NAME,
    PARAM_ASR_MODEL_NAME_DEFAULT,
)
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import AsrResponse, AsrRequest, AsrStreamingResponse
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class BaseAsrNode(QueueNode, ABC):
    """Base class for all ASR nodes in Continuum."""

    _asr_publisher: Publisher[AsrResponse]
    _asr_streaming_publisher: Publisher[AsrStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_parameters(self) -> None:
        """Register the ASR node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_ASR_MODEL_NAME,
            PARAM_ASR_MODEL_NAME_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

    def register_publishers(self) -> None:
        """Register the ASR response publishers."""
        self._asr_publisher = self.create_publisher(AsrResponse, TOPIC_ASR_RESPONSE, QOS_DEPTH_DEFAULT)
        self._asr_streaming_publisher = self.create_publisher(
            AsrStreamingResponse, TOPIC_ASR_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the ASR request subscriber."""
        self.create_subscription(AsrRequest, TOPIC_ASR_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: AsrRequest) -> None:
        """Handle incoming ASR requests."""
        self.get_logger().info(f"ASR request received with session_id: {msg.session_id}")
        sdk_request = ContinuumAsrRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumAsrResponse], sdk_request: ContinuumAsrRequest) -> None:
        """Handle the result of an ASR request."""
        try:
            sdk_response = future.result()
            self.publish_asr_response(sdk_response)
        except Exception as e:
            self.publish_asr_response(
                ContinuumAsrResponse(
                    session_id=sdk_request.session_id, error_code=ERROR_CODE_UNEXPECTED, error_message=str(e)
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(
        self, streaming_response: ContinuumAsrStreamingResponse, sdk_request: ContinuumAsrRequest
    ) -> None:
        """Handle a streaming result from an ASR request."""
        self.publish_asr_streaming_response(streaming_response)

    #
    # Custom publishing methods
    #

    def publish_asr_response(self, sdk_response: ContinuumAsrResponse) -> None:
        if sdk_response.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"ASR response error: {sdk_response}")
        else:
            self.get_logger().info(f"ASR response: {sdk_response}")
        response = AsrResponse()
        set_message_fields(response, sdk_response.model_dump())
        self._asr_publisher.publish(response)

    def publish_asr_streaming_response(self, sdk_streaming_response: ContinuumAsrStreamingResponse) -> None:
        self.get_logger().info(f"ASR streaming response: {sdk_streaming_response}")
        response = AsrStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self._asr_streaming_publisher.publish(response)

    #
    # Properties
    #

    @property
    def model_name(self) -> str:
        """Get the model name parameter."""
        return self._get_str_param(PARAM_ASR_MODEL_NAME)
