# Continuum CLI

Once you install the [server](SERVER.md), a `continuum` CLI will be available to interact with the system.
You can interact with the different subsystems either directly (`asr`, `llm`, `tts` commands) or through the WebSocket
connection (`ws` command).

## Help

```bash
continuum --help
continuum asr --help
continuum llm --help
continuum tts --help
continuum agent --help
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
