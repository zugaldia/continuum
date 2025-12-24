from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.asr import FasterWhisperAsrClient
from continuum.asr.models import FasterWhisperAsrOptions
from continuum_core.asr.base_asr_node import BaseAsrNode


class FasterWhisperAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("fasterwhisper_asr_node")
        self.set_node_info(name="Faster Whisper ASR Node", description="Fast inference engine for Whisper")
        self._client = FasterWhisperAsrClient(options=FasterWhisperAsrOptions())
        self.get_logger().info("Faster Whisper ASR node initialized.")

    def on_shutdown(self) -> None:
        """Clean up Faster Whisper ASR node resources."""
        self.get_logger().info("Faster Whisper ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            fasterwhisper_asr_node = FasterWhisperAsrNode()
            rclpy.spin(fasterwhisper_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
