import rclpy
from rclpy.executors import ExternalShutdownException

from continuum.mic import GStreamerMicOptions, ContinuumMicStreamingResponse, GStreamerMicInterface
from continuum_core.mic.base_mic_node import BaseMicNode


class GStreamerMicNode(BaseMicNode):
    def __init__(self):
        super().__init__("gstreamer_mic_node")
        self.set_node_info(name="GStreamer Microphone Node", description="GStreamer microphone service")

        options = GStreamerMicOptions()
        self._mic_interface = GStreamerMicInterface(options=options, streaming_callback=self._on_audio)
        self.get_logger().info("GStreamer microphone node initialized.")

    def _on_audio(self, streaming_response: ContinuumMicStreamingResponse):
        self.publish_mic_streaming_response(streaming_response)

    def on_shutdown(self) -> None:
        """Clean up microphone resources."""
        self.get_logger().info("GStreamer microphone node shutting down.")
        self._mic_interface.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            gstreamer_mic_node = GStreamerMicNode()
            rclpy.spin(gstreamer_mic_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
