"""CLI argument parsing utilities."""

from pathlib import Path
from typing import Optional

import typer


# Common argument definitions
PATH_OPTION = typer.Option(
    Path("."),
    "--path", "-p",
    help="Path to project to validate",
    exists=True,
    file_okay=False,
    resolve_path=True,
)

VERBOSE_OPTION = typer.Option(
    False,
    "--verbose", "-v",
    help="Enable verbose output",
)

FORMAT_OPTION = typer.Option(
    "table",
    "--format", "-f",
    help="Output format (table, json, yaml)",
)

LIMIT_OPTION = typer.Option(
    20,
    "--limit", "-l",
    help="Maximum number of results",
    min=1,
    max=100,
)


def validate_path(path: Path) -> Path:
    """Validate project path exists and is a directory."""
    if not path.exists():
        raise typer.BadParameter(f"Path does not exist: {path}")
    if not path.is_dir():
        raise typer.BadParameter(f"Path is not a directory: {path}")
    return path


def validate_run_id(run_id: str) -> str:
    """Validate run ID is a valid UUID."""
    from uuid import UUID
    try:
        UUID(run_id)
        return run_id
    except ValueError:
        raise typer.BadParameter(f"Invalid run ID format: {run_id}")


def validate_format(format: str) -> str:
    """Validate output format."""
    valid_formats = ["table", "json", "yaml"]
    if format.lower() not in valid_formats:
        raise typer.BadParameter(f"Invalid format. Must be one of: {valid_formats}")
    return format.lower()
