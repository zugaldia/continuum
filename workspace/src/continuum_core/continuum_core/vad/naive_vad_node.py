from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import (
    PARAM_NAIVE_RMS_THRESHOLD,
    PARAM_NAIVE_RMS_THRESHOLD_DEFAULT,
)
from continuum.vad import NaiveVadClient
from continuum.vad.models import NaiveVadOptions
from continuum_core.vad.base_vad_node import BaseVadNode


class NaiveVadNode(BaseVadNode):
    def __init__(self) -> None:
        super().__init__("naive_vad_node")
        self.set_node_info(name="Naive VAD Node", description="Naive voice activity detection service using RMS.")

        # Read parameters and create options
        rms_threshold = self._get_double_param(PARAM_NAIVE_RMS_THRESHOLD)
        options = NaiveVadOptions(rms_threshold=rms_threshold)

        try:
            self._executor = NaiveVadClient(options=options, streaming_callback=self.handle_streaming_result)
            self.get_logger().info(f"Naive VAD node initialized: {options}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Naive VAD node: {e}")

    def on_shutdown(self) -> None:
        """Clean up Naive VAD node resources."""
        self.get_logger().info("Naive VAD node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_NAIVE_RMS_THRESHOLD,
            PARAM_NAIVE_RMS_THRESHOLD_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            naive_vad_node = NaiveVadNode()
            rclpy.spin(naive_vad_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
