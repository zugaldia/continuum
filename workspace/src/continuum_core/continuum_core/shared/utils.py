"""Shared utility functions for continuum_core nodes."""

from datetime import datetime
from pathlib import Path
from typing import Optional


def create_timestamped_filename(prefix: str, extension: str, directory: Optional[Path] = None) -> Path:
    """Create a unique filename with a timestamp."""
    if directory is None:
        directory = Path("/tmp")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.{extension}"
    return directory / filename
