from typing import Any, cast

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.asr import FasterWhisperAsrClient
from continuum.asr.models import FasterWhisperAsrOptions
from continuum.constants import PARAM_FASTERWHISPER_DEVICE, PARAM_FASTERWHISPER_DEVICE_DEFAULT
from continuum_core.asr.base_asr_node import BaseAsrNode


class FasterWhisperAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("fasterwhisper_asr_node")
        self.set_node_info(name="Faster Whisper ASR Node", description="Fast inference engine for Whisper")

        # Read parameters and create options
        model_name = self.model_name
        device = self._get_str_param(PARAM_FASTERWHISPER_DEVICE)
        download_root = str(self.storage_path)
        options = FasterWhisperAsrOptions(
            model_name=model_name,
            device=device,
            download_root=download_root,
        )

        try:
            self._executor = FasterWhisperAsrClient(options=options, streaming_callback=self.handle_streaming_result)
            if self.debug_mode:
                client = cast(FasterWhisperAsrClient, self._executor)
                self.get_logger().info(f"Available models: {client.get_available_models()}")
                self.get_logger().info(f"Supported languages: {client.get_supported_languages()}")
            self.get_logger().info(f"Faster Whisper ASR node initialized: {options}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Faster Whisper ASR node: {e}")

    def on_shutdown(self) -> None:
        """Clean up Faster Whisper ASR node resources."""
        self.get_logger().info("Faster Whisper ASR node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_FASTERWHISPER_DEVICE,
            PARAM_FASTERWHISPER_DEVICE_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            fasterwhisper_asr_node = FasterWhisperAsrNode()
            rclpy.spin(fasterwhisper_asr_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
