# TTS Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/tts/models.py`

string session_id     # Unique identifier for the TTS session
string audio_path     # Path to the generated audio file
int32 error_code      # Error code (0 = no error)
string error_message  # Error message (default: "All systems nominal")
