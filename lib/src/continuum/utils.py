"""Utility functions for the Continuum SDK."""

import os
import time
import uuid
import wave
from datetime import datetime
from pathlib import Path
from typing import Optional

import mistune
from bs4 import BeautifulSoup

from continuum.constants import (
    DEFAULT_AUDIO_CHANNELS,
    DEFAULT_AUDIO_SAMPLE_RATE,
    DEFAULT_AUDIO_SAMPLE_WIDTH,
    CONTINUUM_ID,
)


def generate_unique_id() -> str:
    """Generate a unique ID."""
    return str(uuid.uuid4())


def generate_timestamp() -> int:
    """Generate a timestamp for request/response model classes using nanosecond precision."""
    return time.time_ns()


def generate_order_id() -> int:
    """Generate a sortable order ID, which is simply using the timestamp value."""
    return generate_timestamp()


def compute_elapsed_ms(start_timestamp: int, end_timestamp: Optional[int] = None) -> float:
    """Compute elapsed time in milliseconds between two timestamps. End defaults to the current time if not provided."""
    if end_timestamp is None:
        end_timestamp = generate_timestamp()
    return (end_timestamp - start_timestamp) / 1e6


def is_empty(s: str | None) -> bool:
    """Check if a string is empty."""
    return s is None or s.strip() == ""


def none_if_empty(s: str | None) -> str | None:
    """Return the string if not empty, None if empty."""
    return None if is_empty(s) else s


def get_data_path() -> Path:
    # Used by Docker Compose
    continuum_data_path = os.environ.get("CONTINUUM_DATA_PATH")
    if not is_empty(continuum_data_path):
        return Path(continuum_data_path)  # type: ignore[arg-type]

    # XDG data directory path (https://specifications.freedesktop.org/basedir/latest/)
    xdg_data_home = os.environ.get("XDG_DATA_HOME")
    if not is_empty(xdg_data_home):
        return Path(xdg_data_home) / CONTINUUM_ID  # type: ignore[arg-type]

    # Typically this is /home/$USER/.local/share/continuum
    return Path.home() / ".local" / "share" / CONTINUUM_ID


def create_timestamped_filename(prefix: str, extension: str, directory: Optional[Path] = None) -> Path:
    """Create a unique filename with a timestamp."""
    if directory is None:
        directory = Path("/tmp")

    # Include microseconds to avoid collisions
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{prefix}_{timestamp}.{extension}"
    return directory / filename


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


def load_wav_file(input_path: Path) -> tuple[bytes, int, int, int]:
    """Load a WAV file and return audio data and metadata (audio_data, sample_rate, channels, sample_width)."""
    with wave.open(str(input_path), "rb") as wav_file:
        audio_data = wav_file.readframes(wav_file.getnframes())
        sample_rate = wav_file.getframerate()
        channels = wav_file.getnchannels()
        sample_width = wav_file.getsampwidth()
    return audio_data, sample_rate, channels, sample_width


def strip_markdown(markdown_text: str) -> str:
    """LLMs tend to be Markdown heavy, which doesn't play well with TTS. This converts Markdown to plain text while
    preserving the line structure so that TTS engines can generate audio line by line (for lower latency)."""
    lines = markdown_text.splitlines()
    result_lines = []
    for line in lines:
        if is_empty(line):
            result_lines.append("")  # Preserve blank lines
        else:
            # Convert line to HTML and strip tags
            html = mistune.html(line)
            soup = BeautifulSoup(html, "html.parser")
            text = soup.get_text(separator=" ", strip=True)
            result_lines.append(text)
    return "\n".join(result_lines)
