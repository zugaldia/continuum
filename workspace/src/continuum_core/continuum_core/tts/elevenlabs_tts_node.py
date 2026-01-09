from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.executors import ExternalShutdownException

from continuum.constants import (
    PARAM_ELEVENLABS_API_KEY,
    PARAM_ELEVENLABS_API_KEY_DEFAULT,
    PARAM_ELEVENLABS_VOICE_ID,
    PARAM_ELEVENLABS_VOICE_ID_DEFAULT,
)
from continuum.tts import ElevenLabsTtsClient
from continuum.tts.models import ElevenLabsTtsOptions
from continuum_core.tts.base_tts_node import BaseTtsNode


class ElevenLabsTtsNode(BaseTtsNode):
    def __init__(self) -> None:
        super().__init__("elevenlabs_tts_node")
        self.set_node_info(name="ElevenLabs TTS Node", description="ElevenLabs natural-sounding TTS service")

        # Read parameters and create options
        model_name = self.model_name
        api_key = self._get_str_param(PARAM_ELEVENLABS_API_KEY)
        voice_id = self._get_str_param(PARAM_ELEVENLABS_VOICE_ID)
        options = ElevenLabsTtsOptions(
            model_name=model_name,
            api_key=api_key,
            voice_id=voice_id,
        )

        try:
            self._executor = ElevenLabsTtsClient(options=options, streaming_callback=self.handle_streaming_result)
            self.get_logger().info(f"ElevenLabs TTS node initialized: {options}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ElevenLabs TTS node: {e}")

    def on_shutdown(self) -> None:
        """Clean up ElevenLabs TTS node resources."""
        self.get_logger().info("ElevenLabs TTS node shutting down.")
        if self._executor is not None:
            self._executor.shutdown()
        super().on_shutdown()

    def register_parameters(self) -> None:
        """Register custom node parameters."""
        super().register_parameters()
        self.declare_parameter(
            PARAM_ELEVENLABS_API_KEY,
            PARAM_ELEVENLABS_API_KEY_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )
        self.declare_parameter(
            PARAM_ELEVENLABS_VOICE_ID,
            PARAM_ELEVENLABS_VOICE_ID_DEFAULT,
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING),
        )


def main(args: Any = None) -> None:
    try:
        with rclpy.init(args=args):
            elevenlabs_tts_node = ElevenLabsTtsNode()
            rclpy.spin(elevenlabs_tts_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
