# TTS Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/tts/models.py`

int64 timestamp       # Timestamp in nanoseconds
string session_id     # Unique identifier for the TTS session
int32 error_code      # Error code (0 = no error)
string error_message  # Error message (default: "All systems nominal")
bool is_initial       # True if this is the first message in the stream
bool is_final         # True if this is the last message in the stream
int64 order_id        # Sortable order ID (nanosecond timestamp)
uint8[] audio_data    # Raw PCM audio data as a byte array
string format         # Audio format (only PCM is supported at the moment)
uint8 channels        # Number of audio channels (typically 1 for mono)
uint32 sample_rate    # Audio sample rate in Hz (typically 16000 for speech)
uint8 sample_width    # Sample width in bytes (typically 2 for 16-bit audio)
