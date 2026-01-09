"""TTS command implementation."""

import asyncio

import typer

from continuum.audio import AudioIO
from continuum.constants import (
    NODE_TTS_KOKORO,
    NODE_TTS_ELEVENLABS,
    PARAM_ELEVENLABS_VOICE_ID_DEFAULT,
    DEFAULT_MODEL_NAME_ELEVENLABS,
)
from continuum.tts import ContinuumTtsClient, KokoroTtsClient, ElevenLabsTtsClient
from continuum.tts.models import ContinuumTtsRequest, ContinuumTtsStreamingResponse, ElevenLabsTtsOptions


def tts_command(
    text: str = typer.Argument(..., help="Text to synthesize to speech"),
    provider: str = typer.Option(NODE_TTS_KOKORO, help="TTS provider to use"),
    language: str = typer.Option("", help="Language code in ISO-639-1 format (e.g. 'en'), empty for auto-detection"),
    api_key: str = typer.Option("", help="API key for cloud providers (ElevenLabs)"),
    voice_id: str = typer.Option(PARAM_ELEVENLABS_VOICE_ID_DEFAULT, help="Voice ID for TTS providers"),
    model_name: str = typer.Option(DEFAULT_MODEL_NAME_ELEVENLABS, help="Model name for TTS providers"),
) -> None:
    """Synthesize text to speech using TTS."""
    typer.echo(f"Synthesizing text using {provider} provider...")

    def streaming_callback(streaming_response: ContinuumTtsStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(f"Received audio chunk: {len(streaming_response.audio_chunk)} bytes")

    client: ContinuumTtsClient
    if provider == NODE_TTS_KOKORO:
        client = KokoroTtsClient(streaming_callback=streaming_callback)
    elif provider == NODE_TTS_ELEVENLABS:
        if not api_key:
            typer.echo("Error: --api-key is required for ElevenLabs provider", err=True)
            raise typer.Exit(code=1)
        options = ElevenLabsTtsOptions(
            api_key=api_key,
            voice_id=voice_id,
            model_name=model_name,
        )
        client = ElevenLabsTtsClient(options=options, streaming_callback=streaming_callback)
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    request = ContinuumTtsRequest(text=text, language=language)

    try:
        response = asyncio.run(client.execute_request(request))
        if response.error_code != 0:
            typer.echo(f"Error during synthesis: {response.error_message}", err=True)
            raise typer.Exit(code=1)

        # Save audio data to file
        audio_path = AudioIO.save_tmp_wav_file(
            audio_data=response.get_audio_bytes(),
            sample_rate=response.sample_rate,
            channels=response.channels,
            sample_width=response.sample_width,
        )
        typer.echo(f"Audio saved to: {audio_path}")
    except Exception as e:
        typer.echo(f"Error during synthesis: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
