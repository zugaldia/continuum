"""

Fake ASR Node for testing.

This is a fake test double - a working implementation designed for testing rather than
production use. Fakes provide real behavior but in a simplified way (similar to an in-memory
database). Fakes are the preferred approach for the Continuum project.

"""

from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.asr.fake_asr_client import FakeAsrClient
from continuum_core.asr.base_asr_node import BaseAsrNode


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


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            fake_asr_node = FakeAsrNode()
            rclpy.spin(fake_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
