"""WebSocket command implementation."""

import asyncio
from contextlib import asynccontextmanager
from typing import Callable

import typer

from continuum.agent.models import ContinuumAgentRequest, ContinuumAgentResponse, ContinuumAgentStreamingResponse
from continuum.apps.models import (
    DICTATION_STATUS_COMPLETED,
    ContinuumDictationRequest,
    ContinuumDictationResponse,
    ContinuumDictationStreamingResponse,
)
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrResponse, ContinuumAsrStreamingResponse
from continuum.client import ContinuumClient
from continuum.constants import (
    ERROR_CODE_SUCCESS,
    NODE_AGENT_PYDANTIC,
    NODE_ASR_FASTERWHISPER,
    NODE_LLM_OLLAMA,
    NODE_TTS_KOKORO,
    PROFILE_LOCAL,
    DEFAULT_AUDIO_FORMAT,
)
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmResponse, ContinuumLlmStreamingResponse
from continuum.models import EchoRequest, EchoResponse
from continuum.tts.models import ContinuumTtsRequest, ContinuumTtsResponse, ContinuumTtsStreamingResponse
from continuum.utils import generate_unique_id, is_empty, create_timestamped_filename, save_wav_file, load_wav_file

app = typer.Typer()

# Client connection defaults
DEFAULT_HOST = "localhost"
DEFAULT_PORT = 9090


@asynccontextmanager
async def continuum_client_connection(host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
    """Context manager for the Continuum client connection lifecycle."""
    client = ContinuumClient(host=host, port=port)
    try:
        await client.connect()
        yield client
    finally:
        client.disconnect()


def run_async_command(async_func: Callable) -> None:
    """Run an async function with proper error handling."""
    try:
        asyncio.run(async_func())
    except KeyboardInterrupt:
        typer.echo("\nInterrupted by user\n")
        raise typer.Exit(code=130)


def create_response_waiter():
    """Create an event and callback helper for waiting on responses."""
    event = asyncio.Event()
    loop = asyncio.get_running_loop()

    def set_event():
        loop.call_soon_threadsafe(event.set)

    return event, set_event


@app.command(name="echo")
def echo_command(
    message: str = typer.Argument(..., help="Message to echo"),
) -> None:
    """Send an echo request and wait for the response."""
    session_id = generate_unique_id()
    typer.echo(f"Sending echo request: {message}")
    typer.echo(f"Session ID: {session_id}")

    async def run_echo():
        response_received, set_response_received = create_response_waiter()
        received_message = None

        def on_echo_response(response: EchoResponse) -> None:
            nonlocal received_message
            typer.echo(f"Response: {response}")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            received_message = response.message
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_echo(on_echo_response)
                request = EchoRequest(session_id=session_id, message=message)
                client.publish_echo(request)
                await asyncio.wait_for(response_received.wait(), timeout=10.0)
                typer.echo(f"Echo message: {received_message}")
        except Exception as e:
            typer.echo(f"Error during echo request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_echo)


@app.command(name="asr")
def asr_command(
    audio_file: str = typer.Argument(..., help="Path to the audio file"),
    node_name: str = typer.Option(NODE_ASR_FASTERWHISPER, help="ASR node name"),
    language: str = typer.Option("", help="Language code in ISO-639-1 format (e.g. 'en'), empty for auto-detection"),
) -> None:
    """Send an ASR request and wait for the response."""
    from pathlib import Path

    audio_path = Path(audio_file)
    if not audio_path.exists():
        typer.echo(f"Error: Audio file not found: {audio_path}", err=True)
        raise typer.Exit(code=1)

    session_id = generate_unique_id()
    typer.echo(f"Sending ASR request to {node_name}: {audio_path}")
    typer.echo(f"Session ID: {session_id}")

    async def run_asr():
        response_received, set_response_received = create_response_waiter()
        received_transcript = None

        def on_asr_streaming_response(streaming_response: ContinuumAsrStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_asr_response(response: ContinuumAsrResponse) -> None:
            nonlocal received_transcript
            typer.echo(f"Final response: transcription='{response.transcription}'")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            received_transcript = response.transcription
            set_response_received()

        try:
            # Load audio file and populate audio_data
            audio_data, sample_rate, channels, sample_width = load_wav_file(audio_path)
            request = ContinuumAsrRequest(session_id=session_id, language=language)
            request.set_audio_bytes(audio_data)
            request.sample_rate = sample_rate
            request.channels = channels
            request.sample_width = sample_width
            request.format = DEFAULT_AUDIO_FORMAT

            async with continuum_client_connection() as client:
                client.subscribe_asr_streaming_response(node_name, on_asr_streaming_response)
                client.subscribe_asr_response(node_name, on_asr_response)
                client.publish_asr_request(node_name, request)
                await asyncio.wait_for(response_received.wait(), timeout=30.0)
                typer.echo(f"ASR transcript: {received_transcript}")
        except Exception as e:
            typer.echo(f"Error during ASR request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_asr)


@app.command(name="tts")
def tts_command(
    text: str = typer.Argument(..., help="Text to convert to speech"),
    node_name: str = typer.Option(NODE_TTS_KOKORO, help="TTS node name"),
    language: str = typer.Option("", help="Language code in ISO-639-1 format (e.g. 'en'), empty for default"),
) -> None:
    """Send a TTS request and wait for the response."""
    session_id = generate_unique_id()
    typer.echo(f"Sending TTS request to {node_name}: {text}")
    typer.echo(f"Session ID: {session_id}")

    async def run_tts():
        response_received, set_response_received = create_response_waiter()
        received_response = None

        def on_tts_streaming_response(streaming_response: ContinuumTtsStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_tts_response(response: ContinuumTtsResponse) -> None:
            nonlocal received_response
            typer.echo(f"Final response: audio_data length={len(response.audio_data)}")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            received_response = response
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_tts_streaming_response(node_name, on_tts_streaming_response)
                client.subscribe_tts_response(node_name, on_tts_response)
                request = ContinuumTtsRequest(session_id=session_id, text=text, language=language)
                client.publish_tts_request(node_name, request)
                await asyncio.wait_for(response_received.wait(), timeout=30.0)

                # Save audio data to file
                if received_response and received_response.audio_data:
                    audio_path = create_timestamped_filename(node_name, "wav")
                    save_wav_file(
                        audio_data=received_response.get_audio_bytes(),
                        output_path=audio_path,
                        sample_rate=received_response.sample_rate,
                        channels=received_response.channels,
                        sample_width=received_response.sample_width,
                    )
                    typer.echo(f"TTS audio path: {audio_path}")
        except Exception as e:
            typer.echo(f"Error during TTS request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_tts)


@app.command(name="llm")
def llm_command(
    message: str = typer.Argument(..., help="Message to send to the LLM"),
    node_name: str = typer.Option(NODE_LLM_OLLAMA, help="LLM node name"),
) -> None:
    """Send an LLM request and wait for the response."""
    session_id = generate_unique_id()
    typer.echo(f"Sending LLM request to {node_name}: {message}")
    typer.echo(f"Session ID: {session_id}")

    async def run_llm():
        response_received, set_response_received = create_response_waiter()
        received_content = None

        def on_llm_streaming_response(streaming_response: ContinuumLlmStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_llm_response(response: ContinuumLlmResponse) -> None:
            nonlocal received_content
            typer.echo(f"Final response: {response}")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            typer.echo(f"Done reason: {response.done_reason}")
            received_content = response.content_text
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_llm_streaming_response(node_name, on_llm_streaming_response)
                client.subscribe_llm_response(node_name, on_llm_response)
                request = ContinuumLlmRequest(session_id=session_id, content_text=message)
                client.publish_llm_request(node_name, request)
                await asyncio.wait_for(response_received.wait(), timeout=30.0)
                typer.echo(f"LLM content: {received_content}")
        except Exception as e:
            typer.echo(f"Error during LLM request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_llm)


@app.command(name="agent")
def agent_command(
    message: str = typer.Argument(..., help="Message to send to the agent"),
    node_name: str = typer.Option(NODE_AGENT_PYDANTIC, help="Agent node name"),
    state_id: str = typer.Option("", help="State ID to continue a conversation"),
) -> None:
    """Send an agent request and wait for the response."""
    # Generate a state_id if not provided
    if is_empty(state_id):
        state_id = generate_unique_id()

    session_id = generate_unique_id()
    typer.echo(f"Sending agent request to {node_name}: {message}")
    typer.echo(f"Session ID: {session_id}")
    typer.echo(f"State ID: {state_id}")

    async def run_agent():
        response_received, set_response_received = create_response_waiter()
        received_response = None

        def on_agent_streaming_response(streaming_response: ContinuumAgentStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_agent_response(response: ContinuumAgentResponse) -> None:
            nonlocal received_response
            typer.echo(f"Final response: {response}")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            received_response = response
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_agent_streaming_response(node_name, on_agent_streaming_response)
                client.subscribe_agent_response(node_name, on_agent_response)
                request = ContinuumAgentRequest(session_id=session_id, content_text=message, state_id=state_id)
                client.publish_agent_request(node_name, request)
                await asyncio.wait_for(response_received.wait(), timeout=30.0)
                typer.echo(f"Agent response received: {received_response}")
        except Exception as e:
            typer.echo(f"Error during agent request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_agent)


@app.command(name="dictation")
def dictation_command(
    audio_file: str = typer.Argument(..., help="Path to the audio file"),
    profile: str = typer.Option(PROFILE_LOCAL, help="Profile to use (local or cloud)"),
    language: str = typer.Option("", help="Language code in ISO-639-1 format (e.g. 'en'), empty for auto-detection"),
) -> None:
    """Send a dictation request and wait for the response."""
    from pathlib import Path

    audio_path = Path(audio_file)
    if not audio_path.exists():
        typer.echo(f"Error: Audio file not found: {audio_path}", err=True)
        raise typer.Exit(code=1)

    session_id = generate_unique_id()
    typer.echo(f"Sending dictation request with profile={profile}")
    typer.echo(f"Audio path: {audio_path}")
    typer.echo(f"Session ID: {session_id}")

    async def run_dictation():
        response_received, set_response_received = create_response_waiter()
        received_content = None

        def on_dictation_streaming_response(streaming_response: ContinuumDictationStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_dictation_response(response: ContinuumDictationResponse) -> None:
            nonlocal received_content
            typer.echo(f"Response: status={response.status}, content_length={len(response.content_text)}")
            typer.echo(f"Status: {response.status}")

            # Only exit when we reach completed status or encounter an error
            if response.status == DICTATION_STATUS_COMPLETED or response.error_code != ERROR_CODE_SUCCESS:
                typer.echo(f"Error code: {response.error_code}")
                typer.echo(f"Error message: {response.error_message}")
                typer.echo(f"ASR duration: {response.asr_duration_seconds:.2f}s")
                typer.echo(f"LLM duration: {response.llm_duration_seconds:.2f}s")
                typer.echo(f"Total duration: {response.total_duration_seconds:.2f}s")
                received_content = response.content_text
                set_response_received()

        try:
            # Load audio file and populate audio_data
            audio_data, sample_rate, channels, sample_width = load_wav_file(audio_path)
            request = ContinuumDictationRequest(session_id=session_id, language=language)
            request.set_audio_bytes(audio_data)
            request.sample_rate = sample_rate
            request.channels = channels
            request.sample_width = sample_width
            request.format = DEFAULT_AUDIO_FORMAT

            async with continuum_client_connection() as client:
                client.subscribe_dictation_streaming_response(on_dictation_streaming_response, profile)
                client.subscribe_dictation_response(on_dictation_response, profile)
                client.publish_dictation_request(request, profile)
                await asyncio.wait_for(response_received.wait(), timeout=60.0)
                typer.echo(f"Dictation content: {received_content}")
        except Exception as e:
            typer.echo(f"Error during dictation request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_dictation)


if __name__ == "__main__":
    app()
