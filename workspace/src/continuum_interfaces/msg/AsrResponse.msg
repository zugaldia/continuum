# ASR Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/asr/models.py`

string session_id             # Unique identifier for the ASR session
string transcription          # The transcribed text from the audio
string language               # Detected language code (empty string "" if not detected/available)
float64 language_probability  # Probability of the detected language (-1.0 if not available)
