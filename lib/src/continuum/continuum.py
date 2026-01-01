"""Continuum CLI interface."""

import logging

import typer

from continuum.cli.asr import asr_command
from continuum.cli.llm import llm_command
from continuum.cli.tts import tts_command
from continuum.cli.ws import app as ws_app

app = typer.Typer()


@app.callback()
def main() -> None:
    """Continuum CLI."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
    )


app.command(name="asr")(asr_command)
app.command(name="llm")(llm_command)
app.command(name="tts")(tts_command)
app.add_typer(ws_app, name="ws", help="WebSocket operations")


def cli() -> None:
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    cli()
