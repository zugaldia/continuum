# VAD Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/vad/models.py`

int64 timestamp       # Timestamp in nanoseconds
string session_id     # Unique identifier for the VAD session
bool is_initial       # True if this is the first message in the stream
bool is_final         # True if this is the last message in the stream
int64 order_id        # Sortable order ID (nanosecond timestamp)
int32 error_code      # Error code (0 = no error)
string error_message  # Error message (default: "All systems nominal")
string event          # VAD event type (SPEECH_START or SPEECH_END)
