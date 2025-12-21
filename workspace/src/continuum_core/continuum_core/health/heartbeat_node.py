import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher
from std_msgs.msg import String

from continuum.constants import QOS_DEPTH_DEFAULT, SDK_VERSION, TOPIC_HEARTBEAT
from continuum_core.shared.base_node import BaseNode


class HeartbeatNode(BaseNode):
    _publisher: Publisher[String]

    def __init__(self):
        super().__init__("heartbeat_node")
        self.set_node_info(name="Heartbeat Node", description="Publishes periodic messages with system status")

        timer_period_seconds = 1
        self.create_timer(timer_period_seconds, self._timer_callback)

        self.get_logger().info("Heartbeat node initialized.")

    def register_publishers(self):
        """Register the heartbeat publisher."""
        self._publisher = self.create_publisher(String, TOPIC_HEARTBEAT, QOS_DEPTH_DEFAULT)

    def on_shutdown(self):
        """Clean up heartbeat node resources."""
        self.get_logger().info("Heartbeat node shutting down...")

    def _timer_callback(self):
        msg = String()
        msg.data = f"All systems nominal (SDK version v{SDK_VERSION})."
        self._publisher.publish(msg)
        self.get_logger().info(f"Status: {msg.data}")


def main(args=None):
    try:
        with rclpy.init(args=args):
            heartbeat_node = HeartbeatNode()
            rclpy.spin(heartbeat_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
