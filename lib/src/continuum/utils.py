"""Utility functions for the Continuum SDK."""

import uuid


def generate_session_id() -> str:
    """Generate a unique session ID."""
    return str(uuid.uuid4())


def is_empty(s: str | None) -> bool:
    """Check if a string is empty."""
    return s is None or s.strip() == ""


def none_if_empty(s: str | None) -> str | None:
    """Return the string if not empty, None if empty."""
    return None if is_empty(s) else s
