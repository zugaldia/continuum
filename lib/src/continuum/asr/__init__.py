"""ASR (Automatic Speech Recognition) module."""

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.fake_asr_client import FakeAsrClient
from continuum.asr.fasterwhisper_asr_client import FasterWhisperAsrClient

__all__ = [
    "ContinuumAsrClient",
    "FakeAsrClient",
    "FasterWhisperAsrClient",
]
