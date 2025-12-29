"""Continuum SDK constants."""

SDK_NAME = "Continuum SDK"
SDK_VERSION = "0.1.0"

CONTINUUM_NAMESPACE = "continuum"
PATH_INPUT = "input"
PATH_ASR = "asr"
PATH_TTS = "tts"
PATH_LLM = "llm"
PATH_APP = "app"
PATH_HARDWARE = "hardware"

# Profile names
PROFILE_LOCAL = "local"
PROFILE_CLOUD = "cloud"

# Node names
NODE_ASR_FAKE = "fake"
NODE_ASR_FASTERWHISPER = "fasterwhisper"
NODE_ASR_OPENAI = "openai"
NODE_TTS_KOKORO = "kokoro"
NODE_LLM_FAKE = "fake"
NODE_LLM_OLLAMA = "ollama"
NODE_LLM_OPENAI = "openai"
NODE_LLM_GOOGLE = "google"
NODE_APP_DICTATION = "dictation"
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

# Common parameters
PARAM_DEBUG_MODE = "debug_mode"
PARAM_DEBUG_MODE_DEFAULT = False
PARAM_NODE_NAME = "node_name"
PARAM_NODE_NAME_DEFAULT = ""
PARAM_NODE_DESCRIPTION = "node_description"
PARAM_NODE_DESCRIPTION_DEFAULT = ""

# Common ASR parameters
PARAM_ASR_MODEL_NAME = "model_name"
PARAM_ASR_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)

# Default model names for each ASR provider
DEFAULT_MODEL_NAME_FASTERWHISPER = "medium"
DEFAULT_MODEL_NAME_OPENAI_ASR = "gpt-4o-transcribe"

# FasterWhisper ASR parameters
PARAM_FASTERWHISPER_DEVICE = "device"
PARAM_FASTERWHISPER_DEVICE_DEFAULT = "auto"
PARAM_FASTERWHISPER_DOWNLOAD_ROOT = "download_root"
PARAM_FASTERWHISPER_DOWNLOAD_ROOT_DEFAULT = ""

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

# Kokoro TTS parameters
PARAM_KOKORO_DEVICE = "device"
PARAM_KOKORO_DEVICE_DEFAULT = ""

# Common LLM parameters
PARAM_LLM_MODEL_NAME = "model_name"
PARAM_LLM_MODEL_NAME_DEFAULT = ""  # Shared param is generically empty (the concrete values are below)

# Default model names for each LLM provider
DEFAULT_MODEL_NAME_OPENAI = "gpt-5.2"
DEFAULT_MODEL_NAME_GOOGLE = "gemini-3-flash-preview"
DEFAULT_MODEL_NAME_OLLAMA = "gemma3"

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

# Shared parameters across Dictation app and Reachy nodes
PARAM_ASR_NODE = "asr_node"
PARAM_ASR_NODE_DEFAULT = NODE_ASR_FASTERWHISPER
PARAM_LLM_NODE = "llm_node"
PARAM_LLM_NODE_DEFAULT = NODE_LLM_OLLAMA
PARAM_TTS_NODE = "tts_node"
PARAM_TTS_NODE_DEFAULT = NODE_TTS_KOKORO

# Reachy params
PARAM_SYSTEM_PROMPT_PATH = "system_prompt_path"
PARAM_SYSTEM_PROMPT_PATH_DEFAULT = ""

# QoS settings
QOS_DEPTH_DEFAULT = 10

# Error codes
ERROR_CODE_SUCCESS = 0
ERROR_CODE_UNEXPECTED = 1
