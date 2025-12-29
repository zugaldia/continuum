# TTS Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/tts/models.py`

string session_id      # Unique identifier for the TTS session
uint8[] audio_chunk    # Incremental audio data chunk
int32 error_code       # Error code (0 = no error)
string error_message   # Error message (default: "All systems nominal")
