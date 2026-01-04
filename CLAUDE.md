# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Continuum is a middleware that unifies voice and LLM services behind a single WebSocket interface using ROS 2 as the
pubsub framework. It provides pluggable ASR (speech-to-text), LLM, and TTS (text-to-speech) providers with both local
and cloud options.

## Build and Development Commands

### Root-level commands (from project root)
```bash
make install-lib          # Install Python SDK (editable mode)
make uninstall-lib        # Uninstall Python SDK
make lint                 # Run linting on both lib/ and workspace/
make format               # Format code in both lib/ and workspace/
```

### Workspace commands (from workspace/)
```bash
make deps                 # Install ROS dependencies
make build-all            # Build all ROS packages (colcon build --symlink-install)
make test                 # Run ROS tests (colcon test)
make lint                 # Lint workspace code (ruff check + mypy)
make format               # Format workspace code (ruff format)
make clean                # Remove build/install/log directories
make kill-zombies         # Kill orphaned ROS processes
```

### Library commands (from lib/)
```bash
make lint                 # ruff check + mypy
make format               # ruff format
```

### CLI usage
```bash
continuum asr <audio_file> --provider faster-whisper|openai
continuum llm "<prompt>" --provider ollama|openai|google
continuum tts "<text>" --provider kokoro|elevenlabs
continuum agent "<message>" --provider pydantic
continuum ws echo|asr|llm|tts|agent|dictation    # WebSocket commands
```

## Architecture

This project is a monorepo with two main components:

1. **lib/** - Python SDK (`continuum` package)
   - Provider implementations for ASR/LLM/TTS (local invocation without ROS)
   - WebSocket client for connecting to ROS bridge
   - CLI tool
   - Shared constants and models

2. **workspace/** - ROS 2 packages
   - `continuum_interfaces` - Custom ROS message types (AsrRequest, LlmResponse, etc.)
   - `continuum_core` - ROS nodes wrapping SDK providers
   - `continuum_desktop` - Launch files and configuration

### ROS topic naming convention
Topics follow the pattern: `/continuum/<category>/<provider>/<topic_name>`
- Categories: `asr`, `llm`, `tts`, `agent`, `app`, `hardware`, `input`
- Example: `/continuum/llm/ollama/llm_request`

### Node naming convention
Each provider node has a `<provider>_<type>_node` pattern:
- `fasterwhisper_asr_node`, `openai_asr_node`
- `ollama_llm_node`, `openai_llm_node`, `google_llm_node`
- `kokoro_tts_node`, `elevenlabs_tts_node`
- `pydantic_agent_node`

### Configuration
Node parameters are configured in `continuum.yaml` at project root. The file uses ROS 2 parameter format with node
namespaces as keys:
```yaml
/continuum/llm/ollama/ollama_llm:
  ros__parameters:
    host: "http://localhost:11434"
    model_name: "gpt-oss"
```

### Request/Response flow
1. Client publishes to `*_request` topic
2. Provider node processes and publishes to `*_response` topic
3. For streaming: intermediate results go to `*_streaming_response` topic

### Constants
All node names, topic names, and parameter keys are defined in `lib/src/continuum/constants.py`.
Import from here rather than hardcoding strings.

## Code Style

- Line length: 120 characters (configured in pyproject.toml)
- Linting: ruff + mypy
- Python version: 3.12+
- ROS 2 distribution: Kilted
