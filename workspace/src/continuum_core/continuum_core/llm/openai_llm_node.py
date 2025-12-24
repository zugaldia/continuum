from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import (
    PARAM_OPENAI_LLM_API_KEY,
    PARAM_OPENAI_LLM_API_KEY_DEFAULT,
    PARAM_OPENAI_LLM_BASE_URL,
    PARAM_OPENAI_LLM_BASE_URL_DEFAULT,
)
from continuum.llm import OpenAiLlmClient
from continuum.llm.models import OpenAiLlmOptions
from continuum_core.llm.base_llm_node import BaseLlmNode


class OpenAILlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("openai_llm_node")
        self.set_node_info(name="OpenAI LLM Node", description="OpenAI LLM node for cloud inference")

        # Get parameters and create options
        api_key = self._get_str_param(PARAM_OPENAI_LLM_API_KEY)
        base_url = self._get_str_param(PARAM_OPENAI_LLM_BASE_URL)
        options = OpenAiLlmOptions(api_key=api_key, base_url=base_url)

        self._client = OpenAiLlmClient(options=options)
        self.get_logger().info(f"OpenAI LLM node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up OpenAI LLM node resources."""
        self.get_logger().info("OpenAI LLM node shutting down.")
        self._client.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_OPENAI_LLM_API_KEY,
            PARAM_OPENAI_LLM_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_OPENAI_LLM_BASE_URL,
            PARAM_OPENAI_LLM_BASE_URL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            openai_llm_node = OpenAILlmNode()
            rclpy.spin(openai_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
