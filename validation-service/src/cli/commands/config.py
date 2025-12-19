"""CLI config command for managing configuration."""

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

from src.core.config import get_settings


console = Console()


def show_config():
    """Show current configuration."""
    settings = get_settings()

    console.print(Panel.fit(
        "[bold]Current Configuration[/bold]",
        title="Validation Service",
    ))

    table = Table(show_header=True, header_style="bold")
    table.add_column("Setting")
    table.add_column("Value")

    table.add_row("Environment", settings.environment)
    table.add_row("Log Level", settings.log_level)
    table.add_row("Quality Enabled", str(settings.enable_quality_validation))
    table.add_row("Tests Enabled", str(settings.enable_test_execution))
    table.add_row("Deployment Enabled", str(settings.enable_deployment_validation))
    table.add_row("Pipeline Timeout", f"{settings.pipeline_timeout_minutes} min")
    table.add_row("Min Coverage", f"{settings.quality.min_coverage_percentage}%")
    table.add_row("Max Complexity", str(settings.quality.max_cyclomatic_complexity))
    table.add_row("Parallel Tests", str(settings.testing.parallel_execution))
    table.add_row("Flaky Detection", str(settings.testing.flaky_detection_enabled))

    console.print(table)


def validate_config():
    """Validate current configuration."""
    try:
        settings = get_settings()
        console.print("[green]Configuration is valid[/green]")

        # Check for warnings
        warnings = []
        if settings.environment == "development":
            warnings.append("Running in development mode")
        if settings.quality.min_coverage_percentage < 50:
            warnings.append("Coverage threshold is below 50%")

        if warnings:
            console.print("\n[yellow]Warnings:[/yellow]")
            for w in warnings:
                console.print(f"  - {w}")

    except Exception as e:
        console.print(f"[red]Configuration error: {e}[/red]")
        raise typer.Exit(1)
