import wave
from pathlib import Path

from continuum.audio.constants import (
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
)
from continuum.constants import CONTINUUM_ID
from continuum.utils import create_timestamped_filename


class AudioIO:
    @staticmethod
    def save_tmp_wav_file(
        audio_data: bytes,
        sample_rate: int = DEFAULT_AUDIO_SAMPLE_RATE,
        channels: int = DEFAULT_AUDIO_CHANNELS,
        sample_width: int = DEFAULT_AUDIO_SAMPLE_WIDTH,
    ) -> Path:
        """Save raw PCM audio data as a /tmp WAV file and return the path."""
        output_path: Path = create_timestamped_filename(prefix=CONTINUUM_ID, extension="wav")
        AudioIO.save_wav_file(audio_data, output_path, sample_rate, channels, sample_width)
        return output_path

    @staticmethod
    def save_wav_file(
        audio_data: bytes,
        output_path: Path,
        sample_rate: int = DEFAULT_AUDIO_SAMPLE_RATE,
        channels: int = DEFAULT_AUDIO_CHANNELS,
        sample_width: int = DEFAULT_AUDIO_SAMPLE_WIDTH,
    ) -> None:
        """Save raw PCM audio data as a WAV file."""
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(output_path), "wb") as wav_file:
            wav_file.setnchannels(channels)
            wav_file.setsampwidth(sample_width)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(audio_data)

    @staticmethod
    def load_wav_file(input_path: Path) -> tuple[bytes, int, int, int]:
        """Load a WAV file and return audio data and metadata (audio_data, sample_rate, channels, sample_width)."""
        with wave.open(str(input_path), "rb") as wav_file:
            audio_data = wav_file.readframes(wav_file.getnframes())
            sample_rate = wav_file.getframerate()
            channels = wav_file.getnchannels()
            sample_width = wav_file.getsampwidth()
        return audio_data, sample_rate, channels, sample_width
