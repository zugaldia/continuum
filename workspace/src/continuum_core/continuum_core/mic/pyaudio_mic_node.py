import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.mic import PyAudioMicOptions, ContinuumMicStreamingResponse, PyAudioMicInterface
from continuum_core.mic.base_mic_node import BaseMicNode


class PyAudioMicNode(BaseMicNode):
    def __init__(self):
        super().__init__("pyaudio_mic_node")
        self.set_node_info(name="PyAudio Microphone Node", description="PyAudio microphone service")

        options = PyAudioMicOptions()
        self._mic_interface = PyAudioMicInterface(options=options, streaming_callback=self._on_audio)
        self.get_logger().info("PyAudio microphone node initialized.")

    def _on_audio(self, streaming_response: ContinuumMicStreamingResponse):
        self.publish_mic_streaming_response(streaming_response)

    def on_shutdown(self) -> None:
        """Clean up microphone resources."""
        self.get_logger().info("PyAudio microphone node shutting down.")
        self._mic_interface.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            pyaudio_mic_node = PyAudioMicNode()
            rclpy.spin(pyaudio_mic_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
