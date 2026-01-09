import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.mic import PicovoiceMicOptions, ContinuumMicStreamingResponse, PicovoiceMicInterface
from continuum_core.mic.base_mic_node import BaseMicNode


class PicovoiceMicNode(BaseMicNode):
    def __init__(self):
        super().__init__("picovoice_mic_node")
        self.set_node_info(name="Picovoice Microphone Node", description="Picovoice microphone service")

        options = PicovoiceMicOptions()
        self._mic_interface = PicovoiceMicInterface(options=options, streaming_callback=self._on_audio)
        self.get_logger().info("Picovoice microphone node initialized.")

    def _on_audio(self, streaming_response: ContinuumMicStreamingResponse):
        self.publish_mic_streaming_response(streaming_response)

    def on_shutdown(self) -> None:
        """Clean up microphone resources."""
        self.get_logger().info("Picovoice microphone node shutting down.")
        self._mic_interface.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            picovoice_mic_node = PicovoiceMicNode()
            rclpy.spin(picovoice_mic_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
