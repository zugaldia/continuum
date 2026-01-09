import struct
import threading
from typing import Callable, Optional, List

from continuum.audio.constants import (
    AUDIO_FORMAT_PCM,
    AUDIO_SAMPLE_WIDTH_16BIT,
    AUDIO_CHANNELS_MONO,
    AUDIO_FRAME_LENGTH_512,
)
from continuum.mic.mic_interface import MicInterface
from continuum.mic.models import PicovoiceMicOptions, ContinuumMicStreamingResponse
from continuum.models import AudioComponent


class PicovoiceMicInterface(MicInterface):
    def __init__(
        self,
        options: PicovoiceMicOptions,
        streaming_callback: Optional[Callable[[ContinuumMicStreamingResponse], None]] = None,
    ):
        super().__init__()

        try:
            from pvrecorder import PvRecorder
        except ImportError as e:
            self._logger.error(
                "Failed to import the Picovoice recorder library. "
                "Please install it with `pip install continuum[mic]` or equivalent."
            )
            raise e

        self._streaming_callback = streaming_callback
        self._session_id: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._recorder = PvRecorder(frame_length=AUDIO_FRAME_LENGTH_512)
        self._audio_buffer: bytearray = bytearray()

        available_devices = self._recorder.get_available_devices()
        for device_id in range(len(available_devices)):
            self._logger.info(f"Available device (id={device_id}): {available_devices[device_id]}")
        self._logger.info(f"Picovoice recorder v{self._recorder.version} initialized: {options}.")

    def start(self, session_id: str) -> None:
        self._session_id = session_id
        self._audio_buffer = bytearray()
        self._recorder.start()
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._capture_audio, daemon=True)
        self._thread.start()

    def _capture_audio(self) -> None:
        while self._recorder.is_recording and not self._stop_event.is_set():
            frame: List[int] = self._recorder.read()
            audio_bytes: bytes = struct.pack("h" * len(frame), *frame)
            self._audio_buffer.extend(audio_bytes)
            if self._streaming_callback is not None:
                self._streaming_callback(
                    ContinuumMicStreamingResponse(
                        session_id=self._session_id,
                        is_initial=False,  # Not supported
                        is_final=False,  # Not supported
                        audio_data=list(audio_bytes),
                        format=AUDIO_FORMAT_PCM,
                        channels=AUDIO_CHANNELS_MONO,
                        sample_rate=self._recorder.sample_rate,
                        sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
                    )
                )

    def stop(self) -> AudioComponent:
        self._recorder.stop()
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            try:
                self._thread.join(timeout=1.0)
            except Exception as e:
                self._logger.error(f"Error stopping audio capture: {e}")
            finally:
                self._thread = None

        return AudioComponent(
            audio_data=list(self._audio_buffer),
            format=AUDIO_FORMAT_PCM,
            channels=AUDIO_CHANNELS_MONO,
            sample_rate=self._recorder.sample_rate,
            sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
        )

    def mute(self) -> None:
        self._logger.warning("Picovoice recorder does not support mute.")

    def unmute(self) -> None:
        self._logger.warning("Picovoice recorder does not support unmute.")

    def shutdown(self) -> None:
        if self._recorder.is_recording:
            self.stop()
        self._recorder.delete()
