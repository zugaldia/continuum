import os
import platform

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.publisher import Publisher

from continuum.constants import QOS_DEPTH_DEFAULT, SDK_VERSION, TOPIC_HEARTBEAT
from continuum_core.shared.base_node import BaseNode


class HeartbeatNode(BaseNode):
    _publisher: Publisher[DiagnosticArray]

    def __init__(self):
        super().__init__("heartbeat_node")
        self.set_node_info(name="Heartbeat Node", description="Publishes periodic messages with system status")

        self._setup_heartbeat_info()
        self.get_logger().info(f"Heartbeat info: {self._heartbeat_info}")

        timer_period_seconds = 1
        self.create_timer(timer_period_seconds, self._timer_callback)

        self.get_logger().info("Heartbeat node initialized.")

    def register_publishers(self) -> None:
        """Register the heartbeat publisher."""
        self._publisher = self.create_publisher(DiagnosticArray, TOPIC_HEARTBEAT, QOS_DEPTH_DEFAULT)

    def on_shutdown(self) -> None:
        """Clean up heartbeat node resources."""
        self.get_logger().info("Heartbeat node shutting down.")

    def _setup_heartbeat_info(self) -> None:
        self._heartbeat_info: dict[str, str] = {
            "debug_mode": str(self.debug_mode),
            "platform_machine": platform.machine(),
            "platform_node": platform.node(),
            "platform_processor": platform.processor(),
            "platform_release": platform.release(),
            "platform_system": platform.system(),
            "platform_version": platform.version(),
            "python_version": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "ros_version": os.environ.get("ROS_VERSION", ""),
            "continuum_version": SDK_VERSION,
        }

    def _timer_callback(self) -> None:
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.OK
        status.name = self.get_name()
        status.message = "All systems nominal."
        status.values = [KeyValue(key=k, value=v) for k, v in self._heartbeat_info.items()]
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.status = [status]
        self._publisher.publish(diagnostic_array)


def main(args=None):
    try:
        with rclpy.init(args=args):
            heartbeat_node = HeartbeatNode()
            rclpy.spin(heartbeat_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
