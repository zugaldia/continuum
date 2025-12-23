"""ASR command implementation."""

import asyncio
from pathlib import Path

import typer

from continuum.asr import ContinuumAsrClient, FakeAsrClient, FasterWhisperAsrClient
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrStreamingResponse
from continuum.constants import NODE_ASR_FAKE, NODE_ASR_FASTER_WHISPER


def asr_command(
    provider: str = typer.Option(NODE_ASR_FASTER_WHISPER, help="ASR provider to use"),
    audio_file: Path = typer.Argument(..., help="Path to the audio file to transcribe"),
) -> None:
    """Transcribe audio file using ASR."""
    if not audio_file.exists():
        typer.echo(f"Error: Audio file not found: {audio_file}", err=True)
        raise typer.Exit(code=1)

    typer.echo(f"Transcribing {audio_file} using {provider} provider...")

    client: ContinuumAsrClient
    if provider == NODE_ASR_FAKE:
        client = FakeAsrClient()
    elif provider == NODE_ASR_FASTER_WHISPER:
        client = FasterWhisperAsrClient()
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
