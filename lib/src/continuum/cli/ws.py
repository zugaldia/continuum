"""WebSocket command implementation."""

import asyncio
from contextlib import asynccontextmanager
from typing import Callable

import typer

from continuum.apps.models import (
    DICTATION_STATUS_COMPLETED,
    ContinuumDictationRequest,
    ContinuumDictationResponse,
    ContinuumDictationStreamingResponse,
)
from continuum.asr.models import ContinuumAsrRequest, ContinuumAsrResponse, ContinuumAsrStreamingResponse
from continuum.client import ContinuumClient
from continuum.constants import ERROR_CODE_SUCCESS, NODE_ASR_FASTER_WHISPER, NODE_LLM_OLLAMA
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmResponse, ContinuumLlmStreamingResponse
from continuum.utils import generate_session_id

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
    typer.echo(f"Sending echo request: {message}")

    async def run_echo():
        response_received, set_response_received = create_response_waiter()
        received_message = None

        def on_echo_response(msg: str) -> None:
            nonlocal received_message
            received_message = msg
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_echo(on_echo_response)
                client.publish_echo(message)
                await asyncio.wait_for(response_received.wait(), timeout=10.0)
                typer.echo(f"Echo response: {received_message}")
        except Exception as e:
            typer.echo(f"Error during echo request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_echo)


@app.command(name="asr")
def asr_command(
    node_name: str = typer.Option(NODE_ASR_FASTER_WHISPER, help="ASR node name"),
    audio_path: str = typer.Argument(..., help="Path to the audio file"),
) -> None:
    """Send an ASR request and wait for the response."""
    session_id = generate_session_id()
    typer.echo(f"Sending ASR request to {node_name}: {audio_path}")
    typer.echo(f"Session ID: {session_id}")

    async def run_asr():
        response_received, set_response_received = create_response_waiter()
        received_transcript = None

        def on_asr_streaming_response(streaming_response: ContinuumAsrStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_asr_response(response: ContinuumAsrResponse) -> None:
            nonlocal received_transcript
            typer.echo(f"Final response: {response}")
            typer.echo(f"Error code: {response.error_code}")
            typer.echo(f"Error message: {response.error_message}")
            received_transcript = response.transcription
            set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_asr_streaming_response(node_name, on_asr_streaming_response)
                client.subscribe_asr_response(node_name, on_asr_response)
                request = ContinuumAsrRequest(session_id=session_id, audio_path=audio_path)
                client.publish_asr_request(node_name, request)
                await asyncio.wait_for(response_received.wait(), timeout=30.0)
                typer.echo(f"ASR transcript: {received_transcript}")
        except Exception as e:
            typer.echo(f"Error during ASR request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_asr)


@app.command(name="llm")
def llm_command(
    node_name: str = typer.Option(NODE_LLM_OLLAMA, help="LLM node name"),
    message: str = typer.Argument(..., help="Message to send to the LLM"),
) -> None:
    """Send an LLM request and wait for the response."""
    session_id = generate_session_id()
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


@app.command(name="dictation")
def dictation_command(
    asr_node_name: str = typer.Option(NODE_ASR_FASTER_WHISPER, help="ASR node name"),
    llm_node_name: str = typer.Option(NODE_LLM_OLLAMA, help="LLM node name"),
    audio_path: str = typer.Argument(..., help="Path to the audio file"),
) -> None:
    """Send a dictation request and wait for the response."""
    session_id = generate_session_id()
    typer.echo(f"Sending dictation request with ASR={asr_node_name}, LLM={llm_node_name}")
    typer.echo(f"Audio path: {audio_path}")
    typer.echo(f"Session ID: {session_id}")

    async def run_dictation():
        response_received, set_response_received = create_response_waiter()
        received_content = None

        def on_dictation_streaming_response(streaming_response: ContinuumDictationStreamingResponse) -> None:
            typer.echo(f"Streaming response: {streaming_response}")

        def on_dictation_response(response: ContinuumDictationResponse) -> None:
            nonlocal received_content
            typer.echo(f"Response: {response}")
            typer.echo(f"Status: {response.status}")

            # Only exit when we reach completed status or encounter an error
            if response.status == DICTATION_STATUS_COMPLETED or response.error_code != ERROR_CODE_SUCCESS:
                typer.echo(f"Error code: {response.error_code}")
                typer.echo(f"Error message: {response.error_message}")
                received_content = response.content_text
                set_response_received()

        try:
            async with continuum_client_connection() as client:
                client.subscribe_dictation_streaming_response(on_dictation_streaming_response)
                client.subscribe_dictation_response(on_dictation_response)
                request = ContinuumDictationRequest(
                    session_id=session_id, asr_node=asr_node_name, llm_node=llm_node_name, audio_path=audio_path
                )
                client.publish_dictation_request(request)
                await asyncio.wait_for(response_received.wait(), timeout=60.0)
                typer.echo(f"Dictation content: {received_content}")
        except Exception as e:
            typer.echo(f"Error during dictation request: {e}", err=True)
            raise typer.Exit(code=1)

    run_async_command(run_dictation)


if __name__ == "__main__":
    app()
