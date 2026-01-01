"""Utility functions for the Continuum SDK."""

import time
import uuid
from datetime import datetime
from pathlib import Path
from typing import Optional

import mistune
from bs4 import BeautifulSoup


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


def create_timestamped_filename(prefix: str, extension: str, directory: Optional[Path] = None) -> Path:
    """Create a unique filename with a timestamp."""
    if directory is None:
        directory = Path("/tmp")

    # Include microseconds to avoid collisions
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"{prefix}_{timestamp}.{extension}"
    return directory / filename


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
