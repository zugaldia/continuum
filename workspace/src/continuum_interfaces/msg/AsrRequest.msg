# ASR Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/asr/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the ASR session
uint8[] audio_data # Audio data as a byte array
string format      # Only PCM (loss-less) is supported at the moment
uint8 channels     # Typically 1 for mono
uint32 sample_rate # Typically 16000 for speech
uint8 sample_width # Typically 16bits (2 bytes) for speech
string language    # Language code in ISO-639-1 format (e.g. `en`), empty for auto-detection
