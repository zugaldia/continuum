from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.asr import OpenAiAsrClient
from continuum.asr.models import OpenAiAsrOptions
from continuum.constants import (
    PARAM_OPENAI_ASR_API_KEY,
    PARAM_OPENAI_ASR_API_KEY_DEFAULT,
    PARAM_OPENAI_ASR_BASE_URL,
    PARAM_OPENAI_ASR_BASE_URL_DEFAULT,
)
from continuum_core.asr.base_asr_node import BaseAsrNode


class OpenAiAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("openai_asr_node")
        self.set_node_info(name="OpenAI ASR Node", description="OpenAI speech recognition service")

        # Read parameters and create options
        model_name = self.model_name
        api_key = self._get_str_param(PARAM_OPENAI_ASR_API_KEY)
        base_url = self._get_str_param(PARAM_OPENAI_ASR_BASE_URL)
        options = OpenAiAsrOptions(model_name=model_name, api_key=api_key, base_url=base_url)

        self._client = OpenAiAsrClient(options=options)
        self.get_logger().info(f"OpenAI ASR node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up OpenAI ASR node resources."""
        self.get_logger().info("OpenAI ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_OPENAI_ASR_API_KEY,
            PARAM_OPENAI_ASR_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_OPENAI_ASR_BASE_URL,
            PARAM_OPENAI_ASR_BASE_URL_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            openai_asr_node = OpenAiAsrNode()
            rclpy.spin(openai_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
