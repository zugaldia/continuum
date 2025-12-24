from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.asr import OpenAiAsrClient
from continuum.asr.models import OpenAiAsrOptions
from continuum_core.asr.base_asr_node import BaseAsrNode


class OpenAiAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("openai_asr_node")
        self.set_node_info(name="OpenAI ASR Node", description="OpenAI speech recognition service")
        self._client = OpenAiAsrClient(options=OpenAiAsrOptions())
        self.get_logger().info("OpenAI ASR node initialized.")

    def on_shutdown(self) -> None:
        """Clean up OpenAI ASR node resources."""
        self.get_logger().info("OpenAI ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            openai_asr_node = OpenAiAsrNode()
            rclpy.spin(openai_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
