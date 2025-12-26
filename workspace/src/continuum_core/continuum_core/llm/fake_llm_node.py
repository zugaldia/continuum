"""

Fake LLM Node for testing.

"""

from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.llm import FakeLlmClient
from continuum.llm.models import FakeLlmOptions
from continuum_core.llm.base_llm_node import BaseLlmNode


class FakeLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("fake_llm_node")
        self.set_node_info(name="Fake LLM Node", description="Fake LLM node for testing purposes")

        # Get parameters and create options
        model_name = self.model_name
        options = FakeLlmOptions(model_name=model_name)

        self._client = FakeLlmClient(options=options)
        self.get_logger().info(f"Fake LLM node initialized: {options}")

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
