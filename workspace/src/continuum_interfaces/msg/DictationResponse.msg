# Dictation Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/apps/models.py`

string session_id     # Unique identifier for the dictation session
string content_text   # Response content from the LLM
int32 error_code      # Error code (0 = no error)
string error_message  # Error message (default: "All systems nominal")
string status         # Status of the dictation request ("queued", "transcribed", "completed", or empty)
string asr_node       # Name of the ASR node that processed the request
string llm_node       # Name of the LLM node that processed the request
float64 asr_duration_seconds   # Time taken by ASR processing in seconds
float64 llm_duration_seconds   # Time taken by LLM processing in seconds
float64 total_duration_seconds # Total time from request start to completion in seconds
