"""

Fake LLM Node for testing.

"""

from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.llm.fake_llm_client import FakeLlmClient
from continuum_core.llm.base_llm_node import BaseLlmNode


class FakeLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("fake_llm_node")
        self.set_node_info(name="Fake LLM Node", description="Fake LLM node for testing purposes")
        self._client = FakeLlmClient()
        self.get_logger().info("Fake LLM node initialized.")

    def on_shutdown(self) -> None:
        """Clean up fake LLM node resources."""
        self.get_logger().info("Fake LLM node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            fake_llm_node = FakeLlmNode()
            rclpy.spin(fake_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
