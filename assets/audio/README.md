# Audio

These are some audio assets for testing.

Sourced from https://github.com/SYSTRAN/faster-whisper/tree/master/tests/data.

## JFK

The original FLAC file was converted to WAV for testing purposes. It was made mono with a 16 kHz sample rate:

```
ffmpeg -i assets/audio/jfk.flac -ac 1 -ar 16000 assets/audio/jfk.wav
```
