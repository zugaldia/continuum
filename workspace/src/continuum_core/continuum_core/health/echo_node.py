import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher

from continuum.constants import ERROR_CODE_SUCCESS, QOS_DEPTH_DEFAULT, TOPIC_ECHO_REQUEST, TOPIC_ECHO_RESPONSE
from continuum.utils import generate_timestamp
from continuum_core.shared.base_node import BaseNode
from continuum_interfaces.msg import EchoRequest, EchoResponse


class EchoNode(BaseNode):
    _echo_publisher: Publisher[EchoResponse]

    def __init__(self):
        super().__init__("echo_node")
        self.set_node_info(name="Echo Node", description="Listens to echo requests and publishes echo responses")
        self.get_logger().info("Echo node initialized.")

    def register_publishers(self) -> None:
        """Register the echo response publisher."""
        self._echo_publisher = self.create_publisher(EchoResponse, TOPIC_ECHO_RESPONSE, QOS_DEPTH_DEFAULT)

    def register_subscribers(self) -> None:
        """Register the echo request subscriber."""
        self.create_subscription(EchoRequest, TOPIC_ECHO_REQUEST, self._listener_callback, QOS_DEPTH_DEFAULT)

    def on_shutdown(self) -> None:
        """Clean up echo node resources."""
        self.get_logger().info("Echo node shutting down.")

    def _listener_callback(self, msg: EchoRequest) -> None:
        self.get_logger().info(f"Echo: {msg}")
        response = EchoResponse()
        response.timestamp = generate_timestamp()
        response.session_id = msg.session_id
        response.message = msg.message
        response.error_code = ERROR_CODE_SUCCESS
        response.error_message = "All systems nominal"
        self._echo_publisher.publish(response)


def main(args=None):
    try:
        with rclpy.init(args=args):
            echo_node = EchoNode()
            rclpy.spin(echo_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
