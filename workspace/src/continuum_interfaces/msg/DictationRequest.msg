# Dictation Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/apps/models.py`

string session_id  # Unique identifier for the dictation session
string audio_path  # Path to the audio file to transcribe
string asr_node    # ASR node implementation to use (e.g., "faster_whisper", "fake")
string llm_node    # LLM node implementation to use (e.g., "ollama", "claude")
