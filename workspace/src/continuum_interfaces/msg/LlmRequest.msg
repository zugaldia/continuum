# LLM Request Message
# Needs to be kept in sync with the data class in `lib/src/continuum/llm/models.py`

string session_id  # Unique identifier for the LLM session
string state_id  # State identifier for chaining responses into conversation threads (default: empty string)
string system_prompt  # System prompt for the LLM (default: empty string)
string content_text  # Message content to send to the LLM
