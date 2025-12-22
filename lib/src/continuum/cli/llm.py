"""LLM command implementation."""

import asyncio
from enum import Enum

import typer

from continuum.llm.fake_llm_client import FakeLlmClient
from continuum.llm.llm_client import ContinuumLlmClient
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmStreamingResponse
from continuum.llm.ollama_llm_client import OllamaLlmClient


class Provider(str, Enum):
    """Available LLM providers."""

    FAKE = "fake"
    OLLAMA = "ollama"


def llm_command(
        provider: Provider = typer.Option(Provider.OLLAMA, help="LLM provider to use"),
        message: str = typer.Argument(..., help="Message to send to the LLM"),
) -> None:
    """Send message to LLM for completion."""
    typer.echo(f"Sending message to {provider.value} provider...")

    client: ContinuumLlmClient
    if provider == Provider.FAKE:
        client = FakeLlmClient()
    elif provider == Provider.OLLAMA:
        client = OllamaLlmClient()
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    request = ContinuumLlmRequest(content_text=message)

    def streaming_callback(streaming_response: ContinuumLlmStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(streaming_response.content_text)

    try:
        response = asyncio.run(client.execute_request(request, streaming_callback=streaming_callback))
        typer.echo(f"\nFinal response: {response}")
    except Exception as e:
        typer.echo(f"Error during LLM request: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
