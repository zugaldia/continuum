# Echo Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the echo session
string message     # Message to echo back
