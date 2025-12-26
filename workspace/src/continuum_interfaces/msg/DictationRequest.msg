# Dictation Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/apps/models.py`

string session_id  # Unique identifier for the dictation session
string audio_path  # Path to the audio file to transcribe
string language  # Language code in ISO-639-1 format (e.g. `en`), empty for auto-detection
string context  # Custom prompt for transcription context (default: empty)
