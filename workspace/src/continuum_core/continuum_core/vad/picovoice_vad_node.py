from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import (
    PARAM_PICOVOICE_API_KEY,
    PARAM_PICOVOICE_API_KEY_DEFAULT,
    PARAM_PICOVOICE_DEVICE,
    PARAM_PICOVOICE_DEVICE_DEFAULT,
    PARAM_PICOVOICE_LIBRARY_PATH,
    PARAM_PICOVOICE_LIBRARY_PATH_DEFAULT,
    PARAM_PICOVOICE_PROBABILITY_THRESHOLD,
    PARAM_PICOVOICE_PROBABILITY_THRESHOLD_DEFAULT,
)
from continuum.vad import PicovoiceVadClient
from continuum.vad.models import PicovoiceVadOptions
from continuum_core.vad.base_vad_node import BaseVadNode


class PicovoiceVadNode(BaseVadNode):
    def __init__(self) -> None:
        super().__init__("picovoice_vad_node")
        self.set_node_info(name="Picovoice VAD Node", description="Picovoice voice activity detection service")

        # Read parameters and create options
        api_key = self._get_str_param(PARAM_PICOVOICE_API_KEY)
        device = self._get_str_param(PARAM_PICOVOICE_DEVICE)
        library_path = self._get_str_param(PARAM_PICOVOICE_LIBRARY_PATH)
        probability_threshold = self._get_double_param(PARAM_PICOVOICE_PROBABILITY_THRESHOLD)
        options = PicovoiceVadOptions(
            api_key=api_key, device=device, library_path=library_path, probability_threshold=probability_threshold
        )

        try:
            self._executor = PicovoiceVadClient(options=options, streaming_callback=self.handle_streaming_result)
            self.get_logger().info(f"Picovoice VAD node initialized: {options}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Picovoice VAD node: {e}")

    def on_shutdown(self) -> None:
        """Clean up Picovoice VAD node resources."""
        self.get_logger().info("Picovoice VAD node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_PICOVOICE_API_KEY,
            PARAM_PICOVOICE_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_PICOVOICE_DEVICE,
            PARAM_PICOVOICE_DEVICE_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_PICOVOICE_LIBRARY_PATH,
            PARAM_PICOVOICE_LIBRARY_PATH_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_PICOVOICE_PROBABILITY_THRESHOLD,
            PARAM_PICOVOICE_PROBABILITY_THRESHOLD_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            picovoice_vad_node = PicovoiceVadNode()
            rclpy.spin(picovoice_vad_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
