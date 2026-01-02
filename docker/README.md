# Docker Setup

This directory contains the Docker documentation for running Continuum in a containerized environment. The Docker
setup provides a fully self-contained installation with ROS 2, all Python dependencies, and the Continuum workspace
pre-built and ready to use.

Alternatively, you can install Continuum directly on your local machine following [these steps](../SERVER.md).

## What's Included

The Docker container includes:
- ROS 2 Kilted
- Continuum Python SDK with all dependencies
- Pre-built ROS workspace with all Continuum nodes
- ROS bridge server for WebSocket connectivity (port 9090)

The container automatically launches the Continuum desktop environment with the WebSocket bridge,
making all services accessible via `ws://localhost:9090`.

## Getting Started

### Configuration

The container uses the `continuum.yaml` file from the project root for configuration. To modify provider settings 
or add API keys, copy the template and edit it before building the container:

```
cp ./workspace/src/continuum_desktop/config/continuum.yaml continuum.yaml
```

### Build the Docker Image

Build the Continuum Docker image using:

```bash
make docker-build
```

This command runs `docker compose build` and creates an image tagged as `continuum:latest`.

### Start the Container

Start the Continuum container in detached mode:

```bash
make docker-up
```

This launches the container in the background with the WebSocket bridge listening on port 9090.
The container automatically runs `ros2 launch continuum_desktop launch_desktop_with_bridge.py` using the
configuration from `continuum.yaml`.

### View Container Logs

Monitor the container output in real-time:

```bash
make docker-logs
```

This follows the container logs, showing ROS node startup messages and any runtime output. Press Ctrl+C to exit
the log view (the container continues running).

### Stop the Container

Stop and remove the running container:

```bash
make docker-down
```

## Connecting to the Container

Once the container is running, you can interact with Continuum services in several ways:

### Using the CLI

If you have the Continuum CLI installed locally (via `make install-lib`), you can use the WebSocket commands:

```bash
continuum ws echo --message "Hello from Docker"
continuum ws asr --audio-file audio.wav --provider faster-whisper
continuum ws llm --message "Hello" --provider ollama
continuum ws tts --message "Hello" --provider kokoro
continuum ws agent --message "What's the weather?" --provider pydantic
```

### Python Client

For programmatic access, see the [Python client implementation](../lib/src/continuum/client/client.py).

### WebSocket Interface

Connect to the WebSocket bridge at `ws://localhost:9090`. The bridge uses JSON-based communication following
the standard `rosbridge` protocol, which is openly documented. Developers can use one of the
[available client libraries](https://github.com/RobotWebTools/rosbridge_suite?tab=readme-ov-file#clients) or
create their own implementation. See the [Python client implementation](../lib/src/continuum/client/client.py)
for guidance on implementing a custom client.
