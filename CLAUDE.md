# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Continuum is a middleware that unifies voice and LLM services behind a single WebSocket interface.
Built on ROS 2 as its pubsub framework, it provides a pluggable architecture for swapping providers (local vs cloud)
without application changes.

## Build Commands

```bash
# Install Python SDK (editable mode)
make install-lib

# Build ROS 2 workspace
cd workspace && make build-all

# Lint everything (lib + workspace)
make lint

# Format everything
make format
```

## Running the System

```bash
# From workspace/ directory:
make launch-desktop               # Core nodes only
make launch-desktop-with-bridge   # With rosbridge WebSocket (port 9090)
make launch-desktop-with-foxglove # With Foxglove bridge
make launch-desktop-with-fakes    # Core nodes + mock providers for testing

# Kill hung processes
make kill-zombies
```

## Architecture

```
lib/                          # Python SDK and CLI
├── src/continuum/
│   ├── asr/                  # ASR client implementations
│   ├── llm/                  # LLM client implementations
│   ├── apps/                 # Application models
│   ├── client/               # rosbridge WebSocket client
│   └── cli/                  # Typer-based CLI

workspace/                    # ROS 2 workspace
├── src/
│   ├── continuum_interfaces/ # C++ message definitions (.msg files)
│   ├── continuum_core/       # Service nodes (ASR, LLM, Apps)
│   └── continuum_desktop/    # Launch configurations
```

### Node Hierarchy

Most ROS nodes follow this inheritance pattern:
```
rclpy.Node → BaseNode → QueueNode → Base{Asr,Llm}Node → Implementation
```

### Service Implementations

**ASR**: FasterWhisperAsrNode (local), OpenAiAsrNode (cloud), FakeAsrNode (testing)

**LLM**: OllamaLlmNode (local), OpenAiLlmNode (cloud), GoogleLlmNode (cloud), FakeLlmNode (testing)

**Apps**: DictationAppNode - orchestrates ASR + LLM with profile-based routing (local/cloud)

### Topic Naming Convention

```
/{NAMESPACE}/{SERVICE}/{NODE_NAME}/{TOPIC}
Example: /continuum/asr/fasterwhisper/asr_request
```

## Configuration

Runtime parameters are in `continuum.yaml`. Each node has its own parameter block:
```yaml
/continuum/asr/fasterwhisper/fasterwhisper_asr:
  ros__parameters:
    model_name: "distil-large-v3.5"
    device: "cuda"
```

## Code Style

- Line length: 120 characters (ruff)
- Type checking: make lint
- ROS 2 release: kilted
