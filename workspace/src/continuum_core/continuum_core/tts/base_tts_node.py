"""Base class for TTS (Text To Speech) nodes."""

from abc import ABC
from concurrent.futures import Future

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields, message_to_ordereddict

from continuum.tts.models import ContinuumTtsRequest, ContinuumTtsResponse, ContinuumTtsStreamingResponse
from continuum.constants import (
    TOPIC_TTS_RESPONSE,
    TOPIC_TTS_STREAMING_RESPONSE,
    QOS_DEPTH_DEFAULT,
    TOPIC_TTS_REQUEST,
    ERROR_CODE_UNEXPECTED,
    ERROR_CODE_SUCCESS,
    PARAM_TTS_MODEL_NAME,
    PARAM_TTS_MODEL_NAME_DEFAULT,
)
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import TtsResponse, TtsRequest, TtsStreamingResponse
from rcl_interfaces.msg import ParameterDescriptor, ParameterType


class BaseTtsNode(QueueNode, ABC):
    """Base class for all TTS nodes in Continuum."""

    _tts_publisher: Publisher[TtsResponse]
    _tts_streaming_publisher: Publisher[TtsStreamingResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    #
    # Base node methods
    #

    def register_parameters(self) -> None:
        """Register the TTS node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_TTS_MODEL_NAME,
            PARAM_TTS_MODEL_NAME_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )

    def register_publishers(self) -> None:
        """Register the TTS response publishers."""
        self._tts_publisher = self.create_publisher(TtsResponse, TOPIC_TTS_RESPONSE, QOS_DEPTH_DEFAULT)
        self._tts_streaming_publisher = self.create_publisher(
            TtsStreamingResponse, TOPIC_TTS_STREAMING_RESPONSE, QOS_DEPTH_DEFAULT
        )

    def register_subscribers(self) -> None:
        """Register the TTS request subscriber."""
        self.create_subscription(TtsRequest, TOPIC_TTS_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def _listener_callback(self, msg: TtsRequest) -> None:
        """Handle incoming TTS requests."""
        self.get_logger().info(f"TTS request received with session_id: {msg.session_id}")
        sdk_request = ContinuumTtsRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    #
    # Queue node methods
    #

    def handle_result(self, future: Future[ContinuumTtsResponse], sdk_request: ContinuumTtsRequest) -> None:
        """Handle the result of a TTS request."""
        try:
            sdk_response = future.result()
            self.publish_tts_response(sdk_response)
        except Exception as e:
            self.publish_tts_response(
                ContinuumTtsResponse(
                    session_id=sdk_request.session_id, error_code=ERROR_CODE_UNEXPECTED, error_message=str(e)
                )
            )
        finally:
            self.manage_queue(sdk_request)

    def handle_streaming_result(
        self, streaming_response: ContinuumTtsStreamingResponse, sdk_request: ContinuumTtsRequest
    ) -> None:
        """Handle a streaming result from a TTS request."""
        self.publish_tts_streaming_response(streaming_response)

    #
    # Custom publishing methods
    #

    def publish_tts_response(self, sdk_response: ContinuumTtsResponse) -> None:
        if sdk_response.error_code != ERROR_CODE_SUCCESS:
            self.get_logger().error(f"TTS response error: {sdk_response}")
        else:
            self.get_logger().info(f"TTS response: {sdk_response}")
        response = TtsResponse()
        set_message_fields(response, sdk_response.model_dump())
        self._tts_publisher.publish(response)

    def publish_tts_streaming_response(self, sdk_streaming_response: ContinuumTtsStreamingResponse) -> None:
        self.get_logger().debug(f"TTS streaming response: {sdk_streaming_response}")
        response = TtsStreamingResponse()
        set_message_fields(response, sdk_streaming_response.model_dump())
        self._tts_streaming_publisher.publish(response)

    #
    # Properties
    #

    @property
    def model_name(self) -> str:
        """Get the model name parameter."""
        return self._get_str_param(PARAM_TTS_MODEL_NAME)
