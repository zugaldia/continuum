"""Agent command implementation."""

import asyncio

import typer

from continuum.agent import PydanticAgentRunner
from continuum.agent.models import (
    ContinuumAgentRequest,
    ContinuumAgentResponse,
    ContinuumAgentStreamingResponse,
    PydanticAgentOptions,
)
from continuum.constants import NODE_AGENT_PYDANTIC
from continuum.utils import generate_unique_id, is_empty


def agent_command(
    message: str = typer.Argument(..., help="Message to send to the agent"),
    provider: str = typer.Option(NODE_AGENT_PYDANTIC, help="Agent provider to use"),
    state_id: str = typer.Option("", help="State ID to continue a conversation"),
) -> None:
    """Send message to agent for processing."""
    # Generate a state_id if not provided
    if is_empty(state_id):
        state_id = generate_unique_id()

    typer.echo(f"Sending message to {provider} agent provider...")
    typer.echo(f"State ID: {state_id}")

    if provider == NODE_AGENT_PYDANTIC:
        options = PydanticAgentOptions()
        client = PydanticAgentRunner(options=options)
    else:
        typer.echo(f"Error: Unknown provider: {provider}", err=True)
        raise typer.Exit(code=1)

    request = ContinuumAgentRequest(content_text=message, state_id=state_id)

    def streaming_callback(streaming_response: ContinuumAgentStreamingResponse) -> None:
        """Callback to display streaming responses."""
        typer.echo(f"Streaming: {streaming_response}")

    try:
        response: ContinuumAgentResponse = asyncio.run(
            client.execute_request(request, streaming_callback=streaming_callback)
        )
        typer.echo(f"\nFinal response: {response}")
        typer.echo(f"Error code: {response.error_code}")
        typer.echo(f"Error message: {response.error_message}")
    except Exception as e:
        typer.echo(f"Error during agent request: {e}", err=True)
        raise typer.Exit(code=1)
    finally:
        client.shutdown()
