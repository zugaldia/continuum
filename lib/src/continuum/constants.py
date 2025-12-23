"""Continuum SDK constants."""

SDK_NAME = "Continuum SDK"
SDK_VERSION = "0.1.0"

CONTINUUM_NAMESPACE = "continuum"
PATH_ASR = "asr"
PATH_LLM = "llm"
PATH_APP = "app"

# Node names
NODE_ASR_FAKE = "fake"
NODE_ASR_FASTER_WHISPER = "faster_whisper"
NODE_LLM_FAKE = "fake"
NODE_LLM_OLLAMA = "ollama"
NODE_LLM_OPENAI = "openai"
NODE_LLM_GOOGLE = "google"
NODE_APP_DICTATION = "dictation"

# Profile names
PROFILE_LOCAL = "local"
PROFILE_CLOUD = "cloud"

# Topic names
TOPIC_ECHO_REQUEST = "echo_request"
TOPIC_ECHO_RESPONSE = "echo_response"
TOPIC_HEARTBEAT = "heartbeat"
TOPIC_ASR_REQUEST = "asr_request"
TOPIC_ASR_RESPONSE = "asr_response"
TOPIC_ASR_STREAMING_RESPONSE = "asr_streaming_response"
TOPIC_LLM_REQUEST = "llm_request"
TOPIC_LLM_RESPONSE = "llm_response"
TOPIC_LLM_STREAMING_RESPONSE = "llm_streaming_response"
TOPIC_DICTATION_REQUEST = "dictation_request"
TOPIC_DICTATION_RESPONSE = "dictation_response"
TOPIC_DICTATION_STREAMING_RESPONSE = "dictation_streaming_response"

# Parameter names
PARAM_NODE_NAME = "node_name"
PARAM_NODE_DESCRIPTION = "node_description"
PARAM_DEBUG_MODE = "debug_mode"

# QoS settings
QOS_DEPTH_DEFAULT = 10

# Error codes
ERROR_CODE_SUCCESS = 0
ERROR_CODE_UNEXPECTED = 1
