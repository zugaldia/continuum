from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import PARAM_OLLAMA_HOST, PARAM_OLLAMA_HOST_DEFAULT
from continuum.llm import OllamaLlmClient
from continuum.llm.models import OllamaLlmOptions
from continuum_core.llm.base_llm_node import BaseLlmNode


class OllamaLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("ollama_llm_node")
        self.set_node_info(name="Ollama LLM Node", description="Ollama LLM node for local inference")

        # Get parameters and create options
        host = self._get_str_param(PARAM_OLLAMA_HOST)
        model_name = self.model_name
        options = OllamaLlmOptions(host=host, model_name=model_name)

        self._executor = OllamaLlmClient(options=options)
        self.get_logger().info(f"Ollama LLM node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up ollama LLM node resources."""
        self.get_logger().info("Ollama LLM node shutting down.")
        self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_OLLAMA_HOST, PARAM_OLLAMA_HOST_DEFAULT, ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            ollama_llm_node = OllamaLlmNode()
            rclpy.spin(ollama_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
