"""Continuum CLI interface."""

import logging

import typer

from continuum.cli.agent import agent_command
from continuum.cli.asr import asr_command
from continuum.cli.llm import llm_command
from continuum.cli.mic import mic_command
from continuum.cli.tts import tts_command
from continuum.cli.vad import vad_command
from continuum.cli.ws import app as ws_app

app = typer.Typer()


@app.callback()
def main() -> None:
    """Continuum CLI."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
    )


app.command(name="agent")(agent_command)
app.command(name="asr")(asr_command)
app.command(name="llm")(llm_command)
app.command(name="mic")(mic_command)
app.command(name="tts")(tts_command)
app.command(name="vad")(vad_command)
app.add_typer(ws_app, name="ws", help="WebSocket operations")


def cli() -> None:
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    cli()
