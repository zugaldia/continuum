from concurrent.futures import Future
from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException
from rosidl_runtime_py import message_to_ordereddict

from continuum.asr.faster_whisper_client import FasterWhisperClient
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrResponse, FasterWhisperOptions
from continuum_core.asr.base_asr_node import BaseAsrNode
from continuum_interfaces.msg import AsrRequest


class FasterWhisperAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("faster_whisper_asr_node")
        self.set_node_info(name="Faster Whisper ASR Node", description="Fast inference engine for Whisper")
        self._client = FasterWhisperClient(options=FasterWhisperOptions())
        self.get_logger().info("Faster Whisper ASR node initialized.")

    def on_shutdown(self) -> None:
        """Clean up Faster Whisper ASR node resources."""
        self.get_logger().info("Faster Whisper ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()

    def _listener_callback(self, msg: AsrRequest) -> None:
        self.get_logger().info(f"Faster Whisper ASR request received with session_id: {msg.session_id}")
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
            faster_whisper_asr_node = FasterWhisperAsrNode()
            rclpy.spin(faster_whisper_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
