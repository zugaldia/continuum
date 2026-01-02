from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import PARAM_KOKORO_DEVICE, PARAM_KOKORO_DEVICE_DEFAULT
from continuum.tts import KokoroTtsClient
from continuum.tts.models import KokoroTtsOptions
from continuum_core.tts.base_tts_node import BaseTtsNode


class KokoroTtsNode(BaseTtsNode):
    def __init__(self) -> None:
        super().__init__("kokoro_tts_node")
        self.set_node_info(name="Kokoro TTS Node", description="Kokoro open-weight TTS service")

        # Read parameters and create options
        model_name = self.model_name
        device = self._get_str_param(PARAM_KOKORO_DEVICE)
        options = KokoroTtsOptions(
            model_name=model_name,
            device=device,
        )

        self._executor = KokoroTtsClient(options=options)
        self.get_logger().info(f"Kokoro TTS node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up Kokoro TTS node resources."""
        self.get_logger().info("Kokoro TTS node shutting down.")
        self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_KOKORO_DEVICE,
            PARAM_KOKORO_DEVICE_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            kokoro_tts_node = KokoroTtsNode()
            rclpy.spin(kokoro_tts_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
