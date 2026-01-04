# Reachy Mini

![Reachy Mini](https://cdn-uploads.huggingface.co/production/uploads/671faa3a541a76b548647676/uEa13KsL5wtQREVZ1ixwc.png)

Reachy Mini is a fantastic little open-source robot [by the Hugging Face team](https://huggingface.co/blog/reachy-mini).

Continuum has experimental built-in support for natural voice interactions with Reachy Mini. In this setup, no custom
software/app is installed on the robot. Instead, the Continuum server runs on your machine in the same local network
as Reachy and sends/receives data using Reachy's WebRTC bridge (using its Python SDK).

Please note that currently, only the wireless version of Reachy has been tested.

## Prerequisites

- A Reachy Mini robot :-) This should work with the simulator as well, but that has not been validated.
- The Continuum Server must be running using [Docker](docker/README.md)
or the [manual installation instructions](SERVER.md).
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
    agent_node: "pydantic"
    tts_node: "kokoro"

# Reachy Pydantic Agent node (hardware path)
/continuum/hardware/pydantic/pydantic_agent:
  ros__parameters:
    provider_name: "ollama"
    model_name: "gpt-oss"
    base_url: "http://localhost:11434"
    instructions_path: "reachy.md"  # Relative to $HOME/.local/share/continuum/pydantic_agent
```

The `instructions_path` is relative to the agent node storage path
(typically `/home/$USER/.local/share/continuum/pydantic_agent`)

3. (Optional) If you want to test audio playback (down button on the joystick), copy a `test.wav` file to the Reachy
storage path folder. An example is provided in the assets folder, and you can copy it with:

```bash
make copy-test-audio
```

## Usage

If you are launching the server using Docker, the Reachy nodes are launched automatically.

Otherwise, if you followed the manual installation instructions, while the main server is running in one terminal
window, open a second terminal window and launch the Reachy server:

```bash
cd continuum/workspace/
source install/setup.bash
make launch-reachy
```

## Known limitations

In the current implementation, we do not use function calling to determine the Reachy emotion that goes with a
response. Instead, we are using custom prompting to extract one, like in the following example:

```markdown
## RESPONSE EXAMPLES

You can prefix responses with one emotion that reflects your mood.

You can pick one from this list: [amazed], [anxiety], [attentive], [boredom], [calming], [cheerful], [come],
[confused], [contempt], [curious], [dance], [disgusted], [displeased], [downcast], [dying], [electric], [enthusiastic],
[exhausted], [fear], [frustrated], [furious], [go_away], [grateful], [helpful], [impatient], [indifferent],
[inquiring], [irritated], [laughing], [lonely], [lost], [loving], [no], [no_excited], [no_sad], [oops], [proud],
[rage], [relief], [reprimand], [resigned], [sad], [scared], [serenity], [shy], [sleep], [success], [surprised],
[thoughtful], [tired], [uncertain], [uncomfortable], [understanding], [welcoming], [yes], [yes_sad].

Examples:

[amazed] Did you know it takes about 8 minutes for light to reach Earth from the Sun?
[inquiring] Would you like to know more about how robots work?
[laughing] Scientists do not trust atoms because they make up everything.
[no] Pizza isn't a vegetable, no matter how much tomato sauce you add.
[no_sad] Sorry, that's not something Demi can help with.
[yes] That's right, Pluto was reclassified as a dwarf planet.
```

If you're looking for a starting prompt,
[check this one out](https://github.com/pollen-robotics/reachy_mini_conversation_app/blob/develop/src/reachy_mini_conversation_app/prompts/default_prompt.txt). 
