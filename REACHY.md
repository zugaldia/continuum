# Reachy Mini

![Reachy Mini](https://cdn-uploads.huggingface.co/production/uploads/671faa3a541a76b548647676/uEa13KsL5wtQREVZ1ixwc.png)

Reachy Mini is a fantastic little open-source robot [by the Hugging Face team](https://huggingface.co/blog/reachy-mini).

Continuum has experimental built-in support for natural voice interactions with Reachy Mini. In this setup, no custom
software/app is installed on the robot. Instead, the Continuum server runs on your machine in the same local network
as Reachy and sends/receives data using Reachy's WebRTC bridge (using its Python SDK).

Please note that currently, only the wireless version of Reachy has been tested.

## Prerequisites

- A Reachy Mini robot :-) This should work with the simulator as well, but that has not been validated.
- The Continuum Server must be installed using the provided [installation instructions](SERVER.md).
- A standard joystick controller is required to trigger interactions with Reachy. There is currently no support for
VAD or wake word activations (yet).

### Joystick

We use the [8BitDo SN30 Pro Wired Controller](https://shop.8bitdo.com/products/8bitdo-sn30-pro-wired-gamepad-for-switch-pc-retropie-raspberry-pi),
but any joystick supported by the [ROS Joy](https://wiki.ros.org/joy) package should work. Please open an issue if
you encounter any problems with another joystick.

Supported interactions:
- **Select button**: Connect to Reachy Mini wirelessly using the built-in WebRTC server.
- **Start button**: Disconnect from Reachy Mini.
- **Y button**: Start voice interaction.
- **X button**: Finish voice interaction.
- **Up**: Take a photo.
- **Down**: Play a test audio file.
- **Left**: Execute a random emotion move.
- **Right**: Execute a random dance move.

## Installation

1. Install the [Reachy Mini Python SDK](https://github.com/pollen-robotics/reachy_mini). Make sure you go through the
quick start guide and confirm you can connect to Reachy wirelessly using the basic examples.

2. Configure your custom prompt and providers using the `continuum.yaml` file that was created when you set up the
server. For example, for a full local experience with a custom prompt:

```yaml
/continuum/hardware/reachy/reachy:
  ros__parameters:
    asr_node: "fasterwhisper"
    llm_node: "ollama"
    tts_node: "kokoro"
    system_prompt_path: "/path/to/prompt.md"
```

## Usage

While the main server is running in one terminal window, open a second terminal window and launch the Reachy server:

```
cd continuum/workspace/
source install/setup.bash
make launch-reachy
```
