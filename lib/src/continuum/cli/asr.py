"""ASR command implementation."""

import asyncio
from enum import Enum
from pathlib import Path

import typer

from continuum.asr.asr_client import ContinuumAsrClient
from continuum.asr.fake_asr_client import FakeAsrClient
from continuum.asr.faster_whisper_client import FasterWhisperClient
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrStreamingResponse


class Provider(str, Enum):
    """Available ASR providers."""

    FAKE = "fake"
    FASTER_WHISPER = "faster-whisper"


def asr_command(
        provider: Provider = typer.Option(Provider.FASTER_WHISPER, help="ASR provider to use"),
        audio_file: Path = typer.Argument(..., help="Path to the audio file to transcribe"),
) -> None:
    """Transcribe audio file using ASR."""
    if not audio_file.exists():
        typer.echo(f"Error: Audio file not found: {audio_file}", err=True)
        raise typer.Exit(code=1)

    typer.echo(f"Transcribing {audio_file} using {provider.value} provider...")

    client: ContinuumAsrClient
    if provider == Provider.FAKE:
        client = FakeAsrClient()
    elif provider == Provider.FASTER_WHISPER:
        client = FasterWhisperClient()
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    request = ContinuumAsrRequest(audio_path=str(audio_file.absolute()))

    def streaming_callback(streaming_response: ContinuumAsrStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(streaming_response.transcription)

    try:
        response = asyncio.run(client.execute_request(request, streaming_callback=streaming_callback))
        typer.echo(f"\nFinal response: {response}")
    except Exception as e:
        typer.echo(f"Error during transcription: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
