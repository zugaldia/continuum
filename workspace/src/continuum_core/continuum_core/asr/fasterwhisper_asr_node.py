from typing import Any, cast

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.asr import FasterWhisperAsrClient
from continuum.asr.models import FasterWhisperAsrOptions
from continuum.constants import (
    PARAM_FASTERWHISPER_MODEL_SIZE_OR_PATH,
    PARAM_FASTERWHISPER_MODEL_SIZE_OR_PATH_DEFAULT,
    PARAM_FASTERWHISPER_DEVICE,
    PARAM_FASTERWHISPER_DEVICE_DEFAULT,
    PARAM_FASTERWHISPER_DOWNLOAD_ROOT,
    PARAM_FASTERWHISPER_DOWNLOAD_ROOT_DEFAULT,
)
from continuum_core.asr.base_asr_node import BaseAsrNode


class FasterWhisperAsrNode(BaseAsrNode):
    def __init__(self) -> None:
        super().__init__("fasterwhisper_asr_node")
        self.set_node_info(name="Faster Whisper ASR Node", description="Fast inference engine for Whisper")

        # Read parameters and create options
        model_size_or_path = self._get_str_param(PARAM_FASTERWHISPER_MODEL_SIZE_OR_PATH)
        device = self._get_str_param(PARAM_FASTERWHISPER_DEVICE)
        download_root = self._get_str_param(PARAM_FASTERWHISPER_DOWNLOAD_ROOT)
        options = FasterWhisperAsrOptions(
            model_size_or_path=model_size_or_path,
            device=device,
            download_root=download_root,
        )

        self._client = FasterWhisperAsrClient(options=options)
        if self.debug_mode:
            client = cast(FasterWhisperAsrClient, self._client)
            self.get_logger().info(f"Available models: {client.get_available_models()}")
            self.get_logger().info(f"Supported languages: {client.get_supported_languages()}")
        self.get_logger().info(f"Faster Whisper ASR node initialized: {options}")

    def on_shutdown(self) -> None:
        """Clean up Faster Whisper ASR node resources."""
        self.get_logger().info("Faster Whisper ASR node shutting down.")
        self._client.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_FASTERWHISPER_MODEL_SIZE_OR_PATH,
            PARAM_FASTERWHISPER_MODEL_SIZE_OR_PATH_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_FASTERWHISPER_DEVICE,
            PARAM_FASTERWHISPER_DEVICE_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_FASTERWHISPER_DOWNLOAD_ROOT,
            PARAM_FASTERWHISPER_DOWNLOAD_ROOT_DEFAULT,
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
