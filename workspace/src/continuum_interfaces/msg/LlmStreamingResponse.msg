# LLM Streaming Response Message
# Needs to be kept in sync with the data class in `lib/src/continuum/llm/models.py`

string session_id   # Unique identifier for the LLM session
string content_text # Incremental response content from the LLM
