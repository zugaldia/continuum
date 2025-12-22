# ASR Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/asr/models.py`

string session_id      # Unique identifier for the ASR session
string transcription   # Incremental transcription from the audio
