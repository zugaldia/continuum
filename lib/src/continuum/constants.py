"""Continuum SDK constants."""

# SDK info
SDK_NAME = "Continuum SDK"
SDK_VERSION = "0.1.0"

# Safe for file/folder names
CONTINUUM_ID = "continuum"

# Used in launch files
CONTINUUM_NAMESPACE = "continuum"
PATH_INPUT = "input"
PATH_ASR = "asr"
PATH_TTS = "tts"
PATH_LLM = "llm"
PATH_APP = "app"
PATH_AGENT = "agent"
PATH_HARDWARE = "hardware"

# Audio constants
DEFAULT_AUDIO_FORMAT = "pcm"
DEFAULT_AUDIO_CHANNELS = 1
DEFAULT_AUDIO_SAMPLE_RATE = 16000
DEFAULT_AUDIO_SAMPLE_WIDTH = 2

# Profile names
PROFILE_LOCAL = "local"
PROFILE_CLOUD = "cloud"

# Node names
NODE_ASR_FAKE = "fake"
NODE_ASR_FASTERWHISPER = "fasterwhisper"
NODE_ASR_OPENAI = "openai"
NODE_TTS_KOKORO = "kokoro"
NODE_TTS_ELEVENLABS = "elevenlabs"
NODE_LLM_FAKE = "fake"
NODE_LLM_OLLAMA = "ollama"
NODE_LLM_OPENAI = "openai"
NODE_LLM_GOOGLE = "google"
NODE_LLM_MAPGPT = "mapgpt"
NODE_APP_DICTATION = "dictation"
NODE_AGENT_PYDANTIC = "pydantic"
NODE_REACHY = "reachy"

# Topic names
TOPIC_ECHO_REQUEST = "echo_request"
TOPIC_ECHO_RESPONSE = "echo_response"
TOPIC_HEARTBEAT = "heartbeat"
TOPIC_JOYSTICK_BUTTON_EVENT = "joystick_button_event"
TOPIC_JOYSTICK_AXIS_EVENT = "joystick_axis_event"
TOPIC_ASR_REQUEST = "asr_request"
TOPIC_ASR_RESPONSE = "asr_response"
TOPIC_ASR_STREAMING_RESPONSE = "asr_streaming_response"
TOPIC_TTS_REQUEST = "tts_request"
TOPIC_TTS_RESPONSE = "tts_response"
TOPIC_TTS_STREAMING_RESPONSE = "tts_streaming_response"
TOPIC_LLM_REQUEST = "llm_request"
TOPIC_LLM_RESPONSE = "llm_response"
TOPIC_LLM_STREAMING_RESPONSE = "llm_streaming_response"
TOPIC_DICTATION_REQUEST = "dictation_request"
TOPIC_DICTATION_RESPONSE = "dictation_response"
TOPIC_DICTATION_STREAMING_RESPONSE = "dictation_streaming_response"
TOPIC_AGENT_REQUEST = "agent_request"
TOPIC_AGENT_RESPONSE = "agent_response"
TOPIC_AGENT_STREAMING_RESPONSE = "agent_streaming_response"

# Common parameters
PARAM_DEBUG_MODE = "debug_mode"
PARAM_DEBUG_MODE_DEFAULT = False
PARAM_NODE_NAME = "node_name"
PARAM_NODE_NAME_DEFAULT = ""
PARAM_NODE_DESCRIPTION = "node_description"
PARAM_NODE_DESCRIPTION_DEFAULT = ""
PARAM_STORAGE_PATH = "storage_path"
PARAM_STORAGE_PATH_DEFAULT = ""

# Common ASR parameters
PARAM_ASR_MODEL_NAME = "model_name"
PARAM_ASR_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)

# Default model names for each ASR provider
DEFAULT_MODEL_NAME_FASTERWHISPER = "medium"
DEFAULT_MODEL_NAME_OPENAI_ASR = "gpt-4o-transcribe"

# FasterWhisper ASR parameters
PARAM_FASTERWHISPER_DEVICE = "device"
PARAM_FASTERWHISPER_DEVICE_DEFAULT = "auto"

# OpenAI ASR parameters
PARAM_OPENAI_ASR_API_KEY = "api_key"
PARAM_OPENAI_ASR_API_KEY_DEFAULT = ""
PARAM_OPENAI_ASR_BASE_URL = "base_url"
PARAM_OPENAI_ASR_BASE_URL_DEFAULT = ""

# Common TTS parameters
PARAM_TTS_MODEL_NAME = "model_name"
PARAM_TTS_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)

# Default model names for each TTS provider
DEFAULT_MODEL_NAME_KOKORO = "hexgrad/Kokoro-82M"  # A repo ID
DEFAULT_MODEL_NAME_ELEVENLABS = "eleven_multilingual_v2"

# Kokoro TTS parameters
PARAM_KOKORO_DEVICE = "device"
PARAM_KOKORO_DEVICE_DEFAULT = ""

# ElevenLabs TTS parameters
PARAM_ELEVENLABS_API_KEY = "api_key"
PARAM_ELEVENLABS_API_KEY_DEFAULT = ""
PARAM_ELEVENLABS_VOICE_ID = "voice_id"
PARAM_ELEVENLABS_VOICE_ID_DEFAULT = "JBFqnCBsd6RMkjVDRZzb"  # George

# Common LLM parameters
PARAM_LLM_MODEL_NAME = "model_name"
PARAM_LLM_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)

# Default model names for each LLM provider
DEFAULT_MODEL_NAME_OPENAI = "gpt-5.2"
DEFAULT_MODEL_NAME_GOOGLE = "gemini-3-flash-preview"
DEFAULT_MODEL_NAME_OLLAMA = "gpt-oss"

# Ollama LLM parameters
PARAM_OLLAMA_HOST = "host"
PARAM_OLLAMA_HOST_DEFAULT = "http://localhost:11434"

# OpenAI LLM parameters
PARAM_OPENAI_LLM_API_KEY = "api_key"
PARAM_OPENAI_LLM_API_KEY_DEFAULT = ""
PARAM_OPENAI_LLM_BASE_URL = "base_url"
PARAM_OPENAI_LLM_BASE_URL_DEFAULT = ""

# Google LLM parameters
PARAM_GOOGLE_LLM_API_KEY = "api_key"
PARAM_GOOGLE_LLM_API_KEY_DEFAULT = ""

# MapGPT LLM parameters
PARAM_MAPBOX_ACCESS_TOKEN = "access_token"
PARAM_MAPBOX_ACCESS_TOKEN_DEFAULT = ""

# Common Agent parameters
PARAM_AGENT_PROVIDER_NAME = "provider_name"
PARAM_AGENT_PROVIDER_NAME_DEFAULT = "ollama"
PARAM_AGENT_MODEL_NAME = "model_name"
PARAM_AGENT_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)
PARAM_AGENT_API_KEY = "api_key"
PARAM_AGENT_API_KEY_DEFAULT = ""
PARAM_AGENT_BASE_URL = "base_url"
PARAM_AGENT_BASE_URL_DEFAULT = ""
PARAM_AGENT_INSTRUCTIONS_PATH = "instructions_path"
PARAM_AGENT_INSTRUCTIONS_PATH_DEFAULT = ""
PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL = "enable_web_search_tool"
PARAM_AGENT_ENABLE_WEB_SEARCH_TOOL_DEFAULT = False
PARAM_AGENT_ENABLE_WEB_FETCH_TOOL = "enable_web_fetch_tool"
PARAM_AGENT_ENABLE_WEB_FETCH_TOOL_DEFAULT = False
PARAM_AGENT_ENABLE_MEMORY_TOOL = "enable_memory_tool"
PARAM_AGENT_ENABLE_MEMORY_TOOL_DEFAULT = False
PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL = "enable_file_search_tool"
PARAM_AGENT_ENABLE_FILE_SEARCH_TOOL_DEFAULT = False

# Default model names for each Agent provider
DEFAULT_MODEL_NAME_PYDANTIC_AGENT = "gpt-oss"

# Shared parameters across Dictation app and Reachy nodes
PARAM_ASR_NODE = "asr_node"
PARAM_ASR_NODE_DEFAULT = NODE_ASR_FASTERWHISPER
PARAM_LLM_NODE = "llm_node"
PARAM_LLM_NODE_DEFAULT = NODE_LLM_OLLAMA
PARAM_AGENT_NODE = "agent_node"
PARAM_AGENT_NODE_DEFAULT = NODE_AGENT_PYDANTIC
PARAM_TTS_NODE = "tts_node"
PARAM_TTS_NODE_DEFAULT = NODE_TTS_KOKORO

# QoS settings
QOS_DEPTH_DEFAULT = 10

# Error codes
ERROR_CODE_SUCCESS = 0
ERROR_CODE_UNEXPECTED = 1
