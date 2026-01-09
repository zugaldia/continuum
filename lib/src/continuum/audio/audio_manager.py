"""

Centralize common in-memory audio processing operations in one place, using librosa as the backend.
Do not use `audioop` (removed in Python 3.13) or Pydub (largely unmaintained).
We currently assume all audio data is in PCM format.

"""

import librosa
import numpy as np

from continuum.audio.constants import AUDIO_SAMPLE_WIDTH_32BIT, AUDIO_SAMPLE_WIDTH_16BIT


class AudioManager:
    """
    Audio format convention: All methods work with librosa's format where multichannel audio is a 2D array
    with shape (channels, samples). The from_bytes/to_bytes methods handle conversion between interleaved
    byte data and this format. See: https://librosa.org/doc/main/multichannel.html
    """

    @staticmethod
    def from_bytes(audio_data: list[int], sample_width: int, channels: int) -> np.ndarray:
        """If audio_data is multichannel, it's expected to be interleaved (e.g., stereo audio is [L, R, L, R, ...])."""
        audio_bytes = bytes(audio_data)
        if sample_width == AUDIO_SAMPLE_WIDTH_16BIT:  # 2 bytes per sample
            audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
            audio_array = audio_array.astype(np.float32) / np.iinfo(np.int16).max
        elif sample_width == AUDIO_SAMPLE_WIDTH_32BIT:  # 4 bytes per sample
            audio_array = np.frombuffer(audio_bytes, dtype=np.int32)
            audio_array = audio_array.astype(np.float32) / np.iinfo(np.int32).max
        else:
            raise ValueError(f"Unsupported sample width: {sample_width}")

        if channels > 1:
            # Reshape to (channels, samples) if multi-channel
            audio_array = audio_array.reshape((channels, -1), order="F")

        return audio_array

    @staticmethod
    def to_bytes(audio_array: np.ndarray, sample_width: int, channels: int) -> list[int]:
        """If audio_array is multichannel, it's returned as interleaved (e.g., stereo audio is [L, R, L, R, ...])."""
        if channels > 1:
            if audio_array.ndim == 2:
                # Flatten multichannel audio to interleaved
                audio_array = audio_array.reshape(-1, order="F")
            else:
                raise ValueError(f"Expected 2D array for multi-channel audio, got shape {audio_array.shape}")

        if sample_width == AUDIO_SAMPLE_WIDTH_16BIT:  # 2 bytes per sample
            audio_int = (audio_array * np.iinfo(np.int16).max).astype(np.int16)
            return list(audio_int.tobytes())
        elif sample_width == AUDIO_SAMPLE_WIDTH_32BIT:  # 4 bytes per sample
            audio_int = (audio_array * np.iinfo(np.int32).max).astype(np.int32)
            return list(audio_int.tobytes())
        else:
            raise ValueError(f"Unsupported sample width: {sample_width}")

    @staticmethod
    def resample(audio_array: np.ndarray, original_sample_rate: float, target_sample_rate: float) -> np.ndarray:
        return librosa.resample(y=audio_array, orig_sr=original_sample_rate, target_sr=target_sample_rate)

    @staticmethod
    def to_mono(audio_array: np.ndarray) -> np.ndarray:
        return librosa.to_mono(y=audio_array)

    @staticmethod
    def get_duration_in_seconds(audio_array: np.ndarray, sample_rate: float) -> float:
        return librosa.get_duration(y=audio_array, sr=sample_rate)

    @staticmethod
    def get_rms(audio_array: np.ndarray, frame_length: int = 2048) -> float:
        rms_values: np.ndarray = librosa.feature.rms(y=audio_array, frame_length=frame_length)
        return float(np.mean(rms_values))
