from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.llm import GoogleLlmClient
from continuum_core.llm.base_llm_node import BaseLlmNode


class GoogleLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("google_llm_node")
        self.set_node_info(name="Google LLM Node", description="Google Gemini LLM node for cloud inference")
        self._client = GoogleLlmClient()
        self.get_logger().info("Google LLM node initialized.")

    def on_shutdown(self) -> None:
        """Clean up Google LLM node resources."""
        self.get_logger().info("Google LLM node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            google_llm_node = GoogleLlmNode()
            rclpy.spin(google_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
