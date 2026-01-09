from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.vad import SileroVadClient
from continuum.vad.models import SileroVadOptions
from continuum_core.vad.base_vad_node import BaseVadNode


class SileroVadNode(BaseVadNode):
    def __init__(self) -> None:
        super().__init__("silero_vad_node")
        self.set_node_info(name="Silero VAD Node", description="Silero voice activity detection service")

        # Read parameters and create options
        options = SileroVadOptions()

        try:
            self._executor = SileroVadClient(options=options, streaming_callback=self.handle_streaming_result)
            self.get_logger().info("Silero VAD node initialized.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Silero VAD node: {e}")

    def on_shutdown(self) -> None:
        """Clean up Silero VAD node resources."""
        self.get_logger().info("Silero VAD node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            silero_vad_node = SileroVadNode()
            rclpy.spin(silero_vad_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
