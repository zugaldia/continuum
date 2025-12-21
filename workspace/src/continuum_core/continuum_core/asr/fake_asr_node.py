"""

Fake ASR Node for testing.

This is a fake test double - a working implementation designed for testing rather than
production use. Fakes provide real behavior but in a simplified way (similar to an in-memory
database). Fakes are the preferred approach for the Continuum project.

"""

from concurrent.futures import Future
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py import message_to_ordereddict

from continuum.asr.fake_asr_client import FakeAsrClient
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrResponse
from continuum_core.asr.base_asr_node import BaseAsrNode
from continuum_interfaces.msg import AsrRequest


class FakeAsrNode(BaseAsrNode):
    def __init__(self):
        super().__init__("fake_asr_node")
        self.set_node_info(name="Fake ASR Node", description="Fake ASR node for testing purposes")
        self._client = FakeAsrClient()
        self.get_logger().info("Fake ASR node initialized.")

    def on_shutdown(self) -> None:
        """Clean up fake ASR node resources."""
        self.get_logger().info("Fake ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()

    def _listener_callback(self, msg: AsrRequest) -> None:
        self.get_logger().info(f"Fake ASR request received with session_id: {msg.session_id}")
        sdk_request = ContinuumAsrRequest.from_ros(message_to_ordereddict(msg))
        self.receive_request(sdk_request)

    def handle_result(self, future: Future[ContinuumAsrResponse], sdk_request: ContinuumAsrRequest) -> None:
        try:
            sdk_response = future.result()
            self.publish_asr_response(sdk_response)
        except Exception as e:
            self.get_logger().error(f"Error processing request: {e}")
        finally:
            self.manage_queue(sdk_request)


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            fake_asr_node = FakeAsrNode()
            rclpy.spin(fake_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
