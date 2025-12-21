"""Base class for ASR (Automatic Speech Recognition) nodes."""

from abc import abstractmethod, ABC

from rclpy.publisher import Publisher
from rosidl_runtime_py import set_message_fields

from continuum.asr.models import ContinuumAsrResponse
from continuum.constants import TOPIC_ASR_RESPONSE, QOS_DEPTH_DEFAULT, TOPIC_ASR_REQUEST
from continuum_core.shared.queue_node import QueueNode
from continuum_interfaces.msg import AsrResponse, AsrRequest


class BaseAsrNode(QueueNode, ABC):
    """Base class for all ASR nodes in Continuum."""
    _asr_publisher: Publisher[AsrResponse]

    def __init__(self, node_name: str):
        super().__init__(node_name)

    def register_publishers(self) -> None:
        """Register the ASR response publisher."""
        self._asr_publisher = self.create_publisher(AsrResponse, TOPIC_ASR_RESPONSE, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the ASR request subscriber."""
        self.create_subscription(AsrRequest, TOPIC_ASR_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    @abstractmethod
    def _listener_callback(self, msg: AsrRequest) -> None:
        """Handle incoming ASR requests. Must be implemented by subclasses."""
        pass

    def publish_asr_response(self, sdk_response: ContinuumAsrResponse) -> None:
        response = AsrResponse()
        set_message_fields(response, sdk_response.model_dump())
        self.get_logger().info(f"ASR response: {sdk_response}")
        self._asr_publisher.publish(response)
