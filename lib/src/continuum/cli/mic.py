"""Mic command implementation."""

import time

import typer

from continuum.audio import AudioIO
from continuum.constants import NODE_MIC_PICOVOICE, NODE_MIC_PYAUDIO, NODE_MIC_GSTREAMER
from continuum.mic import (
    MicInterface,
    PicovoiceMicOptions,
    PyAudioMicOptions,
    GStreamerMicOptions,
    PicovoiceMicInterface,
    PyAudioMicInterface,
    GStreamerMicInterface,
)
from continuum.utils import generate_unique_id


def mic_command(
    provider: str = typer.Option(NODE_MIC_GSTREAMER, help="Mic provider to use"),
    duration: int = typer.Option(10, help="Recording duration in seconds"),
) -> None:
    """Record audio from the microphone and save to a temporary WAV file."""
    typer.echo(f"Recording audio using {provider} provider for {duration} seconds...")

    # Initialize the appropriate mic interface
    interface: MicInterface
    if provider == NODE_MIC_PICOVOICE:
        options = PicovoiceMicOptions()
        interface = PicovoiceMicInterface(options=options)
    elif provider == NODE_MIC_PYAUDIO:
        options = PyAudioMicOptions()
        interface = PyAudioMicInterface(options=options)
    elif provider == NODE_MIC_GSTREAMER:
        options = GStreamerMicOptions()
        interface = GStreamerMicInterface(options=options)
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    try:
        # Start recording
        session_id = generate_unique_id()
        interface.start(session_id)
        typer.echo(f"Recording started (session: {session_id})...")

        # Wait for the specified duration
        time.sleep(duration)

        # Stop recording and get the audio data
        typer.echo("Stopping recording...")
        audio_component = interface.stop()

        # Save to a temporary WAV file
        output_path = AudioIO.save_tmp_wav_file(
            audio_data=bytes(audio_component.audio_data),
            sample_rate=audio_component.sample_rate,
            channels=audio_component.channels,
            sample_width=audio_component.sample_width,
        )

        typer.echo(f"Audio saved to: {output_path}")
    except Exception as e:
        typer.echo(f"Error during recording: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        interface.shutdown()
