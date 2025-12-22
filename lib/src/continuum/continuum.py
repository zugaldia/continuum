"""Continuum CLI interface."""

import logging

import typer

from continuum.cli.asr import asr_command
from continuum.cli.llm import llm_command

app = typer.Typer()


@app.callback()
def main() -> None:
    """Continuum CLI - Audio and language processing toolkit."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
    )


app.command(name="asr")(asr_command)
app.command(name="llm")(llm_command)


def cli() -> None:
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    cli()
