import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from continuum.constants import QOS_DEPTH_DEFAULT, TOPIC_ECHO
from continuum_core.shared.base_node import BaseNode


class EchoNode(BaseNode):
    def __init__(self):
        super().__init__("echo_node")
        self.set_node_info(name="Echo Node", description="Listens to messages and echoes them to the log")
        self.get_logger().info("Echo node initialized.")

    def register_subscribers(self):
        """Register the echo topic subscriber."""
        self.create_subscription(String, TOPIC_ECHO, self._listener_callback, QOS_DEPTH_DEFAULT)

    def on_shutdown(self):
        """Clean up echo node resources."""
        self.get_logger().info("Echo node shutting down...")

    def _listener_callback(self, msg):
        self.get_logger().info(f"Echo: {msg.data}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            echo_node = EchoNode()
            rclpy.spin(echo_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
