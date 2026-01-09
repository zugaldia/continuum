"""ASR command implementation."""

import asyncio
from pathlib import Path

import typer

from continuum.asr import ContinuumAsrClient, FakeAsrClient, FasterWhisperAsrClient, OpenAiAsrClient
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrStreamingResponse
from continuum.audio import AUDIO_FORMAT_PCM, AudioIO
from continuum.constants import NODE_ASR_FAKE, NODE_ASR_FASTERWHISPER, NODE_ASR_OPENAI


def asr_command(
    audio_file: Path = typer.Argument(..., help="Path to the audio file to transcribe"),
    provider: str = typer.Option(NODE_ASR_FASTERWHISPER, help="ASR provider to use"),
    language: str = typer.Option("", help="Language code in ISO-639-1 format (e.g. 'en'), empty for auto-detection"),
) -> None:
    """Transcribe audio file using ASR."""
    if not audio_file.exists():
        typer.echo(f"Error: Audio file not found: {audio_file}", err=True)
        raise typer.Exit(code=1)

    typer.echo(f"Transcribing {audio_file} using {provider} provider...")

    def streaming_callback(streaming_response: ContinuumAsrStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(streaming_response.transcription)

    client: ContinuumAsrClient
    if provider == NODE_ASR_FAKE:
        client = FakeAsrClient(streaming_callback=streaming_callback)
    elif provider == NODE_ASR_FASTERWHISPER:
        client = FasterWhisperAsrClient(streaming_callback=streaming_callback)
    elif provider == NODE_ASR_OPENAI:
        client = OpenAiAsrClient(streaming_callback=streaming_callback)
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    # Load audio file and populate audio_data
    audio_data, sample_rate, channels, sample_width = AudioIO.load_wav_file(audio_file)
    request = ContinuumAsrRequest(language=language)
    request.set_audio_bytes(audio_data)
    request.sample_rate = sample_rate
    request.channels = channels
    request.sample_width = sample_width
    request.format = AUDIO_FORMAT_PCM

    try:
        response = asyncio.run(client.execute_request(request))
        typer.echo(f"\nFinal response: {response}")
    except Exception as e:
        typer.echo(f"Error during transcription: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
