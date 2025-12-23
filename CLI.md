# Continuum CLI

## Help

```bash
continuum --help
continuum asr --help
continuum llm --help
continuum ws --help
```

## ASR Example

```bash
continuum asr assets/audio/jfk.flac --provider faster-whisper
```

## LLM Example

```bash
continuum llm "What is the capital of Spain?" --provider ollama
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

### Dictation

```bash
continuum ws dictation assets/audio/jfk.flac
```
