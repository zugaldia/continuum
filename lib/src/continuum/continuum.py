"""Continuum CLI interface."""

import logging

import typer

from continuum.cli.asr import asr_command

app = typer.Typer()


@app.callback()
def main() -> None:
    """Continuum CLI - Audio and language processing toolkit."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )


app.command(name="asr")(asr_command)


def cli() -> None:
    """Entry point for the CLI."""
    app()


if __name__ == "__main__":
    cli()
