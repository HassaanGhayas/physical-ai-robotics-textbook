"""CLI flaky-tests command for managing flaky tests."""

from typing import Optional

import typer
from rich.console import Console
from rich.table import Table

from src.core.storage import get_storage
from src.core.models import FlakyStatus


console = Console()


def list_flaky_tests(
    status: Optional[str] = typer.Option(None, "--status", "-s", help="Filter by status"),
    limit: int = typer.Option(20, "--limit", "-l", help="Maximum results"),
):
    """List detected flaky tests."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    if status:
        flaky_tests = [t for t in flaky_tests if t.status.value == status]

    flaky_tests = flaky_tests[:limit]

    if not flaky_tests:
        console.print("[green]No flaky tests detected[/green]")
        raise typer.Exit(0)

    table = Table(title="Flaky Tests")
    table.add_column("Test ID", style="cyan", max_width=50)
    table.add_column("Pass Rate", justify="right")
    table.add_column("Sample Size", justify="right")
    table.add_column("Confidence", justify="right")
    table.add_column("Status")

    for test in flaky_tests:
        status_color = "yellow" if test.status == FlakyStatus.ACTIVE_FLAKY else "green"
        table.add_row(
            test.test_id[:50] + "..." if len(test.test_id) > 50 else test.test_id,
            f"{test.pass_rate * 100:.1f}%",
            str(test.sample_size),
            f"{test.confidence * 100:.1f}%",
            f"[{status_color}]{test.status.value}[/{status_color}]",
        )

    console.print(table)


def update_flaky_test(
    test_id: str = typer.Argument(..., help="Test ID to update"),
    status: str = typer.Option(..., "--status", "-s", help="New status"),
):
    """Update the status of a flaky test."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    test = next((t for t in flaky_tests if t.test_id == test_id), None)

    if not test:
        console.print(f"[red]Flaky test not found: {test_id}[/red]")
        raise typer.Exit(1)

    try:
        new_status = FlakyStatus(status)
    except ValueError:
        console.print(f"[red]Invalid status: {status}[/red]")
        console.print(f"Valid values: {[s.value for s in FlakyStatus]}")
        raise typer.Exit(1)

    test.status = new_status
    storage.save(f"flaky_tests/{test_id}", test.model_dump(mode="json"))

    console.print(f"[green]Updated {test_id} to {status}[/green]")
