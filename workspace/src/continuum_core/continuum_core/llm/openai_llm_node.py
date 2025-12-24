from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.llm import OpenAiLlmClient
from continuum_core.llm.base_llm_node import BaseLlmNode


class OpenAILlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("openai_llm_node")
        self.set_node_info(name="OpenAI LLM Node", description="OpenAI LLM node for cloud inference")
        self._client = OpenAiLlmClient()
        self.get_logger().info("OpenAI LLM node initialized.")

    def on_shutdown(self) -> None:
        """Clean up OpenAI LLM node resources."""
        self.get_logger().info("OpenAI LLM node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            openai_llm_node = OpenAILlmNode()
            rclpy.spin(openai_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
