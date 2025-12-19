"""CLI status command for checking service status."""

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

from src.core.storage import get_storage
from src.core.config import get_settings


console = Console()


def show_status():
    """Show service status and recent activity."""
    settings = get_settings()
    storage = get_storage()

    console.print(Panel.fit(
        "[bold blue]Validation Service Status[/bold blue]",
        title="Status",
    ))

    # Service info
    table = Table(show_header=False)
    table.add_column("Property", style="cyan")
    table.add_column("Value")

    table.add_row("Environment", settings.environment)
    table.add_row("Quality Checks", "[green]Enabled[/green]" if settings.enable_quality_validation else "[red]Disabled[/red]")
    table.add_row("Test Execution", "[green]Enabled[/green]" if settings.enable_test_execution else "[red]Disabled[/red]")
    table.add_row("Deployment Checks", "[green]Enabled[/green]" if settings.enable_deployment_validation else "[red]Disabled[/red]")

    console.print(table)

    # Recent runs
    console.print("\n[bold]Recent Validation Runs[/bold]")

    run = storage.get_latest_run()
    if run:
        runs_table = Table()
        runs_table.add_column("Run ID")
        runs_table.add_column("Status")
        runs_table.add_column("Repository")
        runs_table.add_column("Branch")

        status_color = "green" if run.status.value == "passed" else "red"
        runs_table.add_row(
            str(run.run_id)[:8],
            f"[{status_color}]{run.status.value}[/{status_color}]",
            run.repository,
            run.branch,
        )

        console.print(runs_table)
    else:
        console.print("[dim]No recent runs[/dim]")

    # Flaky tests summary
    console.print("\n[bold]Flaky Tests Summary[/bold]")
    flaky_tests = storage.get_active_flaky_tests()
    if flaky_tests:
        from src.core.models import FlakyStatus
        active = sum(1 for t in flaky_tests if t.status == FlakyStatus.ACTIVE_FLAKY)
        resolved = sum(1 for t in flaky_tests if t.status == FlakyStatus.RESOLVED)
        console.print(f"  Active: [yellow]{active}[/yellow]")
        console.print(f"  Resolved: [green]{resolved}[/green]")
    else:
        console.print("  [dim]No flaky tests tracked[/dim]")
