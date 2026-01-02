"""Agent tools for use with LLM agents."""

from datetime import datetime


def get_date_and_time() -> str:
    """Get the current date and time. Returns a string with the day of the week, date, time, and local timezone."""
    now = datetime.now().astimezone()

    # Format: "Friday, January 02, 2026 at 10:30:52 AM (EST)"
    formatted_time = now.strftime("%A, %B %d, %Y at %I:%M:%S %p (%Z)")
    return formatted_time
