# LLM Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/llm/models.py`

string session_id   # Unique identifier for the LLM session
string state_id  # State identifier for chaining responses into conversation threads (default: empty string)
string content_text # Response content from the LLM
string done_reason  # Reason for completion (e.g., "stop", "length")
int32 error_code    # Error code (0 = no error)
string error_message # Error message (default: "All systems nominal")
