# Continuum Server

This document describes the steps you need to take to run the Continuum WebSocket server locally on your machine.
The following instructions assume Ubuntu 24.04 LTS and might need to be adapted depending on your specific OS.

Alternatively, you can use [Docker](./docker/README.md) to run the server in a container.

## Installation

1. Install ROS 2 Kilted by following the [official instructions](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html).
Install the base package, development tools, and build dependencies:

```bash
sudo apt install ros-kilted-ros-base \
    ros-dev-tools \
    ros-kilted-ament-cmake \
    ros-kilted-ament-cmake-python \
    ros-kilted-rosidl-default-generators \
    ros-kilted-joy
```

Make sure everything is working by running the provided talker/listener example. 

2. Install the ROS 2 WebSocket bridge. This module exposes the ROS system as a standard WebSocket,
allowing you to access it with the provided Python SDK (or build your own implementation). 

```bash
sudo apt install ros-kilted-rosbridge-server
```

3. Install the Continuum Python SDK. This SDK provides a WebSocket client for high-level access to the system and
includes the necessary dependencies to interact with all third-party services (e.g., Ollama, OpenAI, Kokoro).

```bash
cd continuum/
sudo apt install python3-pip
make install-lib
```

This command also installs the `continuum` [CLI](CLI.md) under `/home/$USER/.local/bin/continuum`. You can use this
CLI to test/interact with your local WebSocket server. 

4. Initialize ROS dependencies:

```bash
sudo rosdep init
rosdep update
cd continuum/workspace/
make deps
```

5. Configure the system. Copy the provided YAML configuration template to the root of the project and add your API
keys and model preferences: 

```bash
cp ./workspace/src/continuum_desktop/config/continuum.yaml continuum.yaml
```

Example:

```yaml
/continuum/llm/google/google_llm:
  ros__parameters:
    api_key: "AI..."
    model_name: "gemini-3-pro-preview"
```

6. (Optional) If using Kokoro TTS, install the additional language dependencies:

```bash
sudo apt install espeak-ng
make install-spacy-models
```

## Usage

Once you have completed the installation steps above, you are now ready to launch the server locally:

```bash
cd continuum/workspace/
make build-all
source install/setup.bash
make launch-desktop-with-bridge
```

You will see the ROS system initializing. You can leave this terminal window open for easy access to logs.

Use Ctrl+C when you need to stop the server. 
