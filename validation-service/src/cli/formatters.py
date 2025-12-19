"""CLI output formatting utilities."""

import json
from typing import Any, Optional

from rich.console import Console
from rich.table import Table
from rich.panel import Panel
from rich.syntax import Syntax


console = Console()


def format_status(status: str) -> str:
    """Format status with color."""
    status_colors = {
        "passed": "green",
        "failed": "red",
        "error": "red",
        "warning": "yellow",
        "running": "blue",
        "pending": "dim",
        "skipped": "dim",
    }
    color = status_colors.get(status.lower(), "white")
    return f"[{color}]{status.upper()}[/{color}]"


def format_percentage(value: float, threshold: Optional[float] = None) -> str:
    """Format percentage with color based on threshold."""
    formatted = f"{value:.1f}%"
    if threshold is not None:
        if value >= threshold:
            return f"[green]{formatted}[/green]"
        else:
            return f"[red]{formatted}[/red]"
    return formatted


def format_duration(seconds: float) -> str:
    """Format duration in human-readable form."""
    if seconds < 1:
        return f"{seconds * 1000:.0f}ms"
    elif seconds < 60:
        return f"{seconds:.2f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.0f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        return f"{hours}h {minutes}m"


def format_count(passed: int, total: int) -> str:
    """Format pass count with color."""
    if total == 0:
        return "[dim]0/0[/dim]"
    if passed == total:
        return f"[green]{passed}/{total}[/green]"
    elif passed == 0:
        return f"[red]{passed}/{total}[/red]"
    else:
        return f"[yellow]{passed}/{total}[/yellow]"


def print_json(data: Any, indent: int = 2):
    """Print data as formatted JSON."""
    json_str = json.dumps(data, indent=indent, default=str)
    syntax = Syntax(json_str, "json", theme="monokai")
    console.print(syntax)


def print_table(
    data: list[dict],
    columns: list[tuple[str, str]],
    title: Optional[str] = None,
):
    """Print data as a rich table.

    Args:
        data: List of row dictionaries
        columns: List of (key, header) tuples
        title: Optional table title
    """
    table = Table(title=title)

    for key, header in columns:
        table.add_column(header)

    for row in data:
        values = [str(row.get(key, "")) for key, _ in columns]
        table.add_row(*values)

    console.print(table)


def print_summary_panel(title: str, content: str):
    """Print a summary panel."""
    console.print(Panel(content, title=title))


def print_error(message: str):
    """Print an error message."""
    console.print(f"[red]Error: {message}[/red]")


def print_warning(message: str):
    """Print a warning message."""
    console.print(f"[yellow]Warning: {message}[/yellow]")


def print_success(message: str):
    """Print a success message."""
    console.print(f"[green]✓ {message}[/green]")


def print_info(message: str):
    """Print an info message."""
    console.print(f"[blue]ℹ {message}[/blue]")
