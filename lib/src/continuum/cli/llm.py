"""LLM command implementation."""

import asyncio

import typer

from continuum.constants import NODE_LLM_FAKE, NODE_LLM_OLLAMA, NODE_LLM_OPENAI, NODE_LLM_GOOGLE
from continuum.llm import (
    ContinuumLlmClient,
    FakeLlmClient,
    GoogleLlmClient,
    OllamaLlmClient,
    OpenAiLlmClient,
)
from continuum.llm.models import ContinuumLlmRequest, ContinuumLlmStreamingResponse, ContinuumLlmResponse
from continuum.utils import strip_markdown


def llm_command(
    message: str = typer.Argument(..., help="Message to send to the LLM"),
    provider: str = typer.Option(NODE_LLM_OLLAMA, help="LLM provider to use"),
    state_id: str = typer.Option("", help="State ID for chaining responses into conversation threads"),
) -> None:
    """Send message to LLM for completion."""
    typer.echo(f"Sending message to {provider} provider...")

    client: ContinuumLlmClient
    if provider == NODE_LLM_FAKE:
        client = FakeLlmClient()
    elif provider == NODE_LLM_OLLAMA:
        client = OllamaLlmClient()
    elif provider == NODE_LLM_OPENAI:
        client = OpenAiLlmClient()
    elif provider == NODE_LLM_GOOGLE:
        client = GoogleLlmClient()
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    request = ContinuumLlmRequest(content_text=message, state_id=state_id)

    def streaming_callback(streaming_response: ContinuumLlmStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(streaming_response.content_text)

    try:
        response: ContinuumLlmResponse = asyncio.run(
            client.execute_request(request, streaming_callback=streaming_callback)
        )
        typer.echo(f"\nFinal response: {response}")
        parsed = strip_markdown(response.content_text)
        typer.echo(f"\nFinal response (plain text): {parsed}")
    except Exception as e:
        typer.echo(f"Error during LLM request: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
