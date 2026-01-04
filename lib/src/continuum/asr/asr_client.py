import io
import wave
from abc import ABC

from continuum.asr.models import ContinuumAsrRequest
from continuum.constants import CONTINUUM_ID
from continuum.models import ContinuumExecutor


class ContinuumAsrClient(ContinuumExecutor, ABC):
    """Base class for ASR clients with shared utilities."""

    @staticmethod
    def _create_audio_buffer(request: ContinuumAsrRequest) -> io.BytesIO:
        """Create audio buffer from request with proper WAV formatting."""
        if not request.audio_data:
            raise ValueError("Audio data must be provided in the ASR request.")

        # Create a BytesIO buffer and write a proper WAV file to it
        audio_buffer = io.BytesIO()
        audio_buffer.name = f"{CONTINUUM_ID}.wav"  # Add name attribute - required by some APIs (e.g., OpenAI)

        with wave.open(audio_buffer, "wb") as wav_file:
            wav_file.setnchannels(request.channels)
            wav_file.setsampwidth(request.sample_width)
            wav_file.setframerate(request.sample_rate)
            wav_file.writeframes(request.get_audio_bytes())

        # Reset buffer position to the beginning for reading
        audio_buffer.seek(0)
        return audio_buffer
