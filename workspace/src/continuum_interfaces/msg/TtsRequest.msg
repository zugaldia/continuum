# TTS Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/tts/models.py`

string session_id  # Unique identifier for the TTS session
string text  # Text to synthesize to speech
string language  # Language code in ISO-639-1 format (e.g. `en`), empty for auto-detection
