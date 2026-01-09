"""VAD command implementation."""

import asyncio
import time

import typer

from continuum.vad import ContinuumVadClient, SileroVadClient, PicovoiceVadClient, NaiveVadClient
from continuum.vad.models import (
    ContinuumVadRequest,
    ContinuumVadStreamingResponse,
    SileroVadOptions,
    PicovoiceVadOptions,
    NaiveVadOptions,
)
from continuum.constants import (
    NODE_VAD_SILERO,
    NODE_VAD_PICOVOICE,
    NODE_VAD_NAIVE,
    NODE_MIC_PICOVOICE,
    NODE_MIC_PYAUDIO,
    NODE_MIC_GSTREAMER,
)
from continuum.mic import (
    MicInterface,
    PicovoiceMicOptions,
    PyAudioMicOptions,
    GStreamerMicOptions,
    ContinuumMicStreamingResponse,
    PicovoiceMicInterface,
    PyAudioMicInterface,
    GStreamerMicInterface,
)
from continuum.utils import generate_unique_id


def vad_command(
    mic_provider: str = typer.Option(NODE_MIC_GSTREAMER, help="Microphone provider to use"),
    vad_provider: str = typer.Option(NODE_VAD_SILERO, help="VAD provider to use"),
    duration: int = typer.Option(10, help="Recording duration in seconds"),
    api_key: str = typer.Option("", help="API key for providers that require authentication (e.g., Picovoice)"),
    rms_threshold: float = typer.Option(0.1, help="RMS threshold for naive VAD provider (0.0-1.0)"),
    rms_gain: float = typer.Option(10.0, help="RMS gain multiplier for naive VAD provider"),
    min_speech_frames: int = typer.Option(3, help="Minimum consecutive frames for SPEECH_START (naive VAD)"),
    min_silence_frames: int = typer.Option(5, help="Minimum consecutive frames for SPEECH_END (naive VAD)"),
) -> None:
    """Record audio from the microphone and perform real-time voice activity detection."""
    typer.echo(f"Recording audio using {mic_provider} provider for {duration} seconds...")
    typer.echo(f"Analyzing voice activity using {vad_provider} provider...")

    # Accumulator for audio data
    audio_buffer = bytearray()
    session_id = generate_unique_id()
    chunk_count = 0
    vad_event_count = 0

    def vad_streaming_callback(streaming_response: ContinuumVadStreamingResponse) -> None:
        """Callback to display VAD streaming events."""
        nonlocal vad_event_count
        vad_event_count += 1
        typer.echo(f"[VAD Event {vad_event_count}] {streaming_response}")

    # Initialize VAD client
    vad_client: ContinuumVadClient
    if vad_provider == NODE_VAD_SILERO:
        vad_options = SileroVadOptions()
        vad_client = SileroVadClient(options=vad_options, streaming_callback=vad_streaming_callback)
    elif vad_provider == NODE_VAD_PICOVOICE:
        vad_options = PicovoiceVadOptions(api_key=api_key)
        vad_client = PicovoiceVadClient(options=vad_options, streaming_callback=vad_streaming_callback)
    elif vad_provider == NODE_VAD_NAIVE:
        vad_options = NaiveVadOptions(
            rms_threshold=rms_threshold,
            rms_gain=rms_gain,
            min_speech_frames=min_speech_frames,
            min_silence_frames=min_silence_frames,
        )
        vad_client = NaiveVadClient(options=vad_options, streaming_callback=vad_streaming_callback)
    else:
        typer.echo(f"Error: Unknown VAD provider: {vad_provider}", err=True)
        raise typer.Exit(code=1)

    def mic_streaming_callback(streaming_response: ContinuumMicStreamingResponse) -> None:
        """Callback to process streaming microphone data with VAD."""
        nonlocal chunk_count
        chunk_count += 1

        # Accumulate audio data
        audio_buffer.extend(streaming_response.audio_data)

        # Create a VAD request from the audio chunk
        vad_request = ContinuumVadRequest()
        vad_request.session_id = session_id
        vad_request.audio_data = streaming_response.audio_data
        vad_request.format = streaming_response.format
        vad_request.sample_rate = streaming_response.sample_rate
        vad_request.channels = streaming_response.channels
        vad_request.sample_width = streaming_response.sample_width

        try:
            # Process chunk with VAD and subscribe to streaming events
            asyncio.run(vad_client.execute_request(vad_request))
        except Exception as exception:
            typer.echo(f"Error processing VAD chunk: {exception}", err=True)

    # Initialize microphone interface with streaming callback
    mic_interface: MicInterface
    if mic_provider == NODE_MIC_PICOVOICE:
        mic_options = PicovoiceMicOptions()
        mic_interface = PicovoiceMicInterface(options=mic_options, streaming_callback=mic_streaming_callback)
    elif mic_provider == NODE_MIC_PYAUDIO:
        mic_options = PyAudioMicOptions()
        mic_interface = PyAudioMicInterface(options=mic_options, streaming_callback=mic_streaming_callback)
    elif mic_provider == NODE_MIC_GSTREAMER:
        mic_options = GStreamerMicOptions()
        mic_interface = GStreamerMicInterface(options=mic_options, streaming_callback=mic_streaming_callback)
    else:
        typer.echo(f"Error: Unknown mic provider: {mic_provider}", err=True)
        raise typer.Exit(code=1)

    try:
        # Start recording
        mic_interface.start(session_id)
        typer.echo(f"Recording started (session: {session_id})...")

        # Wait for the specified duration
        time.sleep(duration)

        # Stop recording
        typer.echo("Stopping recording...")
        mic_interface.stop()

        typer.echo(f"\nProcessed {chunk_count} audio chunks")
        typer.echo(f"Detected {vad_event_count} VAD events")
        typer.echo(f"Total audio data: {len(audio_buffer)} bytes")
    except Exception as e:
        typer.echo(f"Error during VAD analysis: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        mic_interface.shutdown()
        if hasattr(vad_client, "shutdown"):
            vad_client.shutdown()
