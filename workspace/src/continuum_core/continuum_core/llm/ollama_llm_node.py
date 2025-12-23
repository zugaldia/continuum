from typing import Any

import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.llm import OllamaLlmClient
from continuum_core.llm.base_llm_node import BaseLlmNode


class OllamaLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("ollama_llm_node")
        self.set_node_info(name="Ollama LLM Node", description="Ollama LLM node for local inference")
        self._client = OllamaLlmClient()
        self.get_logger().info("Ollama LLM node initialized.")

    def on_shutdown(self) -> None:
        """Clean up ollama LLM node resources."""
        self.get_logger().info("Ollama LLM node shutting down.")
        self._client.shutdown()
        super().on_shutdown()


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            ollama_llm_node = OllamaLlmNode()
            rclpy.spin(ollama_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
