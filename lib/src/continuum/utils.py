"""Utility functions for the Continuum SDK."""

import uuid
from datetime import datetime
from pathlib import Path
from typing import Optional

import mistune
from bs4 import BeautifulSoup


def generate_unique_id() -> str:
    """Generate a unique ID."""
    return str(uuid.uuid4())


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
