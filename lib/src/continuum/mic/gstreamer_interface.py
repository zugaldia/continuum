from typing import Optional, Callable, Any

from continuum.audio.constants import (
    AUDIO_FORMAT_PCM,
    AUDIO_SAMPLE_WIDTH_16BIT,
    AUDIO_CHANNELS_MONO,
    AUDIO_SAMPLE_RATE_16KHZ,
)
from continuum.mic.mic_interface import MicInterface
from continuum.mic.models import GStreamerMicOptions, ContinuumMicStreamingResponse
from continuum.models import AudioComponent


class GStreamerMicInterface(MicInterface):
    def __init__(
        self,
        options: GStreamerMicOptions,
        streaming_callback: Optional[Callable[[ContinuumMicStreamingResponse], None]] = None,
    ):
        super().__init__()

        try:
            import gi

            gi.require_version("Gst", "1.0")
            from gi.repository import Gst
        except ImportError as e:
            self._logger.error(
                "Failed to import the GStreamer library. "
                "Please install it with `pip install continuum[mic]` or equivalent."
            )
            raise e

        self._streaming_callback: Optional[Callable[[ContinuumMicStreamingResponse], None]] = streaming_callback
        self._gst_module: Any = Gst
        self._session_id: Optional[str] = None
        self._audio_buffer: bytearray = bytearray()
        self._pipeline: Any = None
        self._app_sink: Any = None

        Gst.init(None)
        self._logger.info(f"GStreamer recorder initialized: {options}.")

    def start(self, session_id: str) -> None:
        self._session_id = session_id
        self._audio_buffer = bytearray()

        pipeline_str = (
            f"autoaudiosrc ! "
            f"audioconvert ! "
            f"audioresample ! "
            f"audio/x-raw,format=S16LE,channels={AUDIO_CHANNELS_MONO},rate={AUDIO_SAMPLE_RATE_16KHZ} ! "
            f"appsink name=sink emit-signals=true"
        )

        self._pipeline = self._gst_module.parse_launch(pipeline_str)
        self._app_sink = self._pipeline.get_by_name("sink")
        self._app_sink.connect("new-sample", self._on_new_sample)
        self._pipeline.set_state(self._gst_module.State.PLAYING)

    def _on_new_sample(self, sink: Any) -> Any:
        """Callback for when a new sample is available from the app sink."""
        sample: Any = sink.emit("pull-sample")
        if sample is None:
            return self._gst_module.FlowReturn.OK

        buffer: Any = sample.get_buffer()
        success: bool
        map_info: Any
        success, map_info = buffer.map(self._gst_module.MapFlags.READ)
        if not success:
            return self._gst_module.FlowReturn.OK

        audio_bytes: bytes = bytes(map_info.data)
        buffer.unmap(map_info)
        self._audio_buffer.extend(audio_bytes)
        if self._streaming_callback is not None:
            self._streaming_callback(
                ContinuumMicStreamingResponse(
                    session_id=self._session_id,
                    is_initial=False,
                    is_final=False,
                    audio_data=list(audio_bytes),
                    format=AUDIO_FORMAT_PCM,
                    channels=AUDIO_CHANNELS_MONO,
                    sample_rate=AUDIO_SAMPLE_RATE_16KHZ,
                    sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
                )
            )

        return self._gst_module.FlowReturn.OK

    def stop(self) -> AudioComponent:
        if self._pipeline is not None:
            self._pipeline.set_state(self._gst_module.State.NULL)
        return AudioComponent(
            audio_data=list(self._audio_buffer),
            format=AUDIO_FORMAT_PCM,
            channels=AUDIO_CHANNELS_MONO,
            sample_rate=AUDIO_SAMPLE_RATE_16KHZ,
            sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
        )

    def mute(self) -> None:
        self._logger.warning("GStreamer recorder does not support mute.")

    def unmute(self) -> None:
        self._logger.warning("GStreamer recorder does not support unmute.")

    def shutdown(self) -> None:
        if self._pipeline is not None:
            self.stop()
