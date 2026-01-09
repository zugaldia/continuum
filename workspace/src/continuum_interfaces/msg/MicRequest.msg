# Mic Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/mic/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the mic session
string action      # Mic action: START, STOP, MUTE, UNMUTE
