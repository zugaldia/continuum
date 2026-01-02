from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import PARAM_GOOGLE_LLM_API_KEY, PARAM_GOOGLE_LLM_API_KEY_DEFAULT
from continuum.llm import GoogleLlmClient
from continuum.llm.models import GoogleLlmOptions
from continuum_core.llm.base_llm_node import BaseLlmNode


class GoogleLlmNode(BaseLlmNode):
    def __init__(self):
        super().__init__("google_llm_node")
        self.set_node_info(name="Google LLM Node", description="Google Gemini LLM node for cloud inference")

        # Get parameters and create options
        api_key = self._get_str_param(PARAM_GOOGLE_LLM_API_KEY)
        model_name = self.model_name
        options = GoogleLlmOptions(api_key=api_key, model_name=model_name)

        self._executor = GoogleLlmClient(options=options)
        self.get_logger().info(f"Google LLM node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up Google LLM node resources."""
        self.get_logger().info("Google LLM node shutting down.")
        self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_GOOGLE_LLM_API_KEY,
            PARAM_GOOGLE_LLM_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            google_llm_node = GoogleLlmNode()
            rclpy.spin(google_llm_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
