# VAD Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/vad/models.py`

int64 timestamp    # Timestamp in nanoseconds
string session_id  # Unique identifier for the VAD session
uint8[] audio_data # Audio data as a byte array
string format      # Only PCM (loss-less) is supported at the moment
uint8 channels     # Typically 1 for mono
uint32 sample_rate # Typically 16000 for speech
uint8 sample_width # Typically 16bits (2 bytes) for speech
