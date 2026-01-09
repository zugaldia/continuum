# Continuum CLI

Once you install the [server](SERVER.md), a `continuum` CLI will be available to interact with the system.
You can interact with the different subsystems either directly (`asr`, `llm`, `tts`, `agent`, `mic`, `vad` commands) or through the WebSocket
connection (`ws` command).

## Help

```bash
continuum --help
continuum asr --help
continuum llm --help
continuum tts --help
continuum agent --help
continuum mic --help
continuum vad --help
continuum ws --help
```

## ASR Example

```bash
continuum asr assets/audio/jfk.flac --provider fasterwhisper
```

## LLM Example

```bash
continuum llm "What is the capital of Spain?" --provider ollama
```

## TTS Example

```bash
continuum tts "Hello world" --provider kokoro
```

## Agent Example

```bash
continuum agent "Do a web search and summarize the latest tech news." --provider pydantic
```

## Mic Example

Record audio from the microphone for 10 seconds (default) and save to a temporary WAV file:

```bash
continuum mic
```

Record for a custom duration:

```bash
continuum mic --duration 5
```

Use a different provider:

```bash
continuum mic --provider pyaudio
```

## WebSocket Examples

### Echo

```bash
continuum ws echo "hello world"
```

### ASR

```bash
continuum ws asr assets/audio/jfk.flac
```

### LLM

```bash
continuum ws llm "What is the capital of France?"
```

### TTS

```bash
continuum ws tts "Hello world"
```

### Agent

```bash
continuum ws agent "Do a web search and summarize the latest tech news."
```

### Dictation

This is what [Speed of Sound](https://github.com/zugaldia/speedofsound) uses under the hood:

```bash
continuum ws dictation assets/audio/jfk.flac
```

### Mic

Record audio from the microphone via WebSocket (requires mic node to be running):

```bash
continuum ws mic --duration 10
```

Use a different mic node:

```bash
continuum ws mic --node-name pyaudio --duration 5
```

### VAD (Voice Activity Detection)

Process an audio file with VAD:

```bash
continuum ws vad assets/audio/jfk.flac
```

Real-time microphone recording with VAD analysis:

```bash
continuum ws mic_vad --duration 10
```

Use different providers:

```bash
continuum ws mic_vad --mic-node pyaudio --vad-node silero --duration 5
```
