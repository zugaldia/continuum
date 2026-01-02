# Agent Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/agent/models.py`

int64 timestamp       # Timestamp in nanoseconds
string session_id     # Unique identifier for the agent session
int32 error_code      # Error code (0 = no error)
string error_message  # Error message (default: "All systems nominal")
string state_id       # State identifier for conversation continuity
string content_text   # Response content from the agent
