"""

Fake ASR Node for testing.

This is a fake test double - a working implementation designed for testing rather than
production use. Fakes provide real behavior but in a simplified way (similar to an in-memory
database). Fakes are the preferred approach for the Continuum project.

"""

from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.asr import FakeAsrClient
from continuum.asr.models import FakeAsrOptions
from continuum_core.asr.base_asr_node import BaseAsrNode


class FakeAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("fake_asr_node")
        self.set_node_info(name="Fake ASR Node", description="Fake ASR node for testing purposes")

        # Read parameters and create options
        model_name = self.model_name
        options = FakeAsrOptions(model_name=model_name)

        try:
            self._executor = FakeAsrClient(options=options)
            self.get_logger().info(f"Fake ASR node initialized: {options}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Fake ASR node: {e}")

    def on_shutdown(self) -> None:
        """Clean up fake ASR node resources."""
        self.get_logger().info("Fake ASR node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
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
