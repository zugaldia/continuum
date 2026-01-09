# Mic Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/mic/models.py`

int64 timestamp        # Timestamp in nanoseconds
string session_id      # Unique identifier for the mic session
bool is_initial        # True if this is the first message in the stream
bool is_final          # True if this is the last message in the stream
int64 order_id         # Sortable order ID (nanosecond timestamp)
int32 error_code       # Error code (0 = no error)
string error_message   # Error message (default: "All systems nominal")
uint8[] audio_data     # Audio data as a byte array
string format          # Audio format (e.g., PCM)
uint8 channels         # Number of audio channels
uint32 sample_rate     # Audio sample rate in Hz
uint8 sample_width     # Sample width in bytes
