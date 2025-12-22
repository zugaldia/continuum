# ASR Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/asr/models.py`

string session_id      # Unique identifier for the ASR session
string transcription   # Incremental transcription from the audio
int32 error_code       # Error code (0 = no error)
string error_message   # Error message (default: "All systems nominal")
