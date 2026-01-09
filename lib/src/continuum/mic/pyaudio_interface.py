from typing import Optional, Callable

from continuum.audio.constants import (
    AUDIO_FORMAT_PCM,
    AUDIO_SAMPLE_WIDTH_16BIT,
    AUDIO_CHANNELS_MONO,
    AUDIO_SAMPLE_RATE_16KHZ,
    AUDIO_FRAME_LENGTH_512,
)
from continuum.mic.mic_interface import MicInterface
from continuum.mic.models import PyAudioMicOptions, ContinuumMicStreamingResponse
from continuum.models import AudioComponent


class PyAudioMicInterface(MicInterface):
    def __init__(
        self,
        options: PyAudioMicOptions,
        streaming_callback: Optional[Callable[[ContinuumMicStreamingResponse], None]] = None,
    ):
        super().__init__()

        try:
            import pyaudio
        except ImportError as e:
            self._logger.error(
                "Failed to import the PyAudio library. "
                "Please install it with `pip install continuum[mic]` or equivalent."
            )
            raise e

        self._streaming_callback = streaming_callback
        self._session_id: Optional[str] = None
        self._pyaudio_module = pyaudio
        self._pyaudio = pyaudio.PyAudio()
        self._stream: Optional[pyaudio.Stream] = None
        self._audio_buffer: bytearray = bytearray()
        self._logger.info(f"PyAudio initialized: {options}.")

    def _audio_callback(self, in_data: bytes, frame_count: int, time_info: dict, status: int) -> tuple:
        """Callback function called by PyAudio when audio data is available."""
        if status:
            self._logger.warning(f"PyAudio status: {status}")

        self._audio_buffer.extend(in_data)
        if self._streaming_callback is not None:
            self._streaming_callback(
                ContinuumMicStreamingResponse(
                    session_id=self._session_id,
                    is_initial=False,
                    is_final=False,
                    audio_data=list(in_data),
                    format=AUDIO_FORMAT_PCM,
                    channels=AUDIO_CHANNELS_MONO,
                    sample_rate=AUDIO_SAMPLE_RATE_16KHZ,
                    sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
                )
            )

        return in_data, self._pyaudio_module.paContinue

    def start(self, session_id: str) -> None:
        self._session_id = session_id
        self._audio_buffer = bytearray()
        self._stream = self._pyaudio.open(
            format=self._pyaudio_module.paInt16,
            channels=AUDIO_CHANNELS_MONO,
            rate=AUDIO_SAMPLE_RATE_16KHZ,
            input=True,
            frames_per_buffer=AUDIO_FRAME_LENGTH_512,
            stream_callback=self._audio_callback,
        )

        self._stream.start_stream()
        self._logger.info(f"Started recording for session {session_id}")

    def stop(self) -> AudioComponent:
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None

        return AudioComponent(
            audio_data=list(self._audio_buffer),
            format=AUDIO_FORMAT_PCM,
            channels=AUDIO_CHANNELS_MONO,
            sample_rate=AUDIO_SAMPLE_RATE_16KHZ,
            sample_width=AUDIO_SAMPLE_WIDTH_16BIT,
        )

    def mute(self) -> None:
        self._logger.warning("PyAudio recorder does not support mute.")

    def unmute(self) -> None:
        self._logger.warning("PyAudio recorder does not support unmute.")

    def shutdown(self) -> None:
        if self._stream is not None:
            self.stop()
        self._pyaudio.terminate()
