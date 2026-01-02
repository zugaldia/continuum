# Agent Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/agent/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the agent session
string state_id    # State identifier for conversation continuity
string content_text  # Message content to send to the agent
