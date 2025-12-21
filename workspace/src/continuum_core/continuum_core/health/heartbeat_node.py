import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from continuum.constants import SDK_VERSION


class HeartbeatNode(Node):
    def __init__(self):
        super().__init__("heartbeat_node")
        self._publisher = self.create_publisher(String, "topic", 10)
        timer_period_seconds = 1
        self.create_timer(timer_period_seconds, self._timer_callback)
        self.get_logger().info("Heartbeat node initialized.")

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
