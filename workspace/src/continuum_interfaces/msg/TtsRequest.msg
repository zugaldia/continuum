# TTS Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/tts/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the TTS session
bool is_initial    # True if this is the first message in the stream
bool is_final      # True if this is the last message in the stream
int64 order_id     # Sortable order ID (nanosecond timestamp)
string text        # Text to synthesize to speech
string language    # Language code in ISO-639-1 format (e.g. `en`), empty for auto-detection
