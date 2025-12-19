"""CLI results command for viewing validation results."""

from typing import Optional
from uuid import UUID

import typer
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

from src.core.storage import get_storage
from src.core.models import ValidationStatus


console = Console()


def view_results(
    run_id: Optional[str] = typer.Argument(None, help="Run ID to view (latest if omitted)"),
    format: str = typer.Option("table", "--format", "-f", help="Output format (table, json)"),
):
    """View validation results."""
    storage = get_storage()

    if run_id:
        run = storage.get_run(UUID(run_id))
        if not run:
            console.print(f"[red]Run not found: {run_id}[/red]")
            raise typer.Exit(1)
    else:
        run = storage.get_latest_run()
        if not run:
            console.print("[yellow]No validation runs found[/yellow]")
            raise typer.Exit(0)

    if format == "json":
        import json
        console.print(json.dumps({
            "run_id": str(run.run_id),
            "status": run.status.value,
            "repository": run.repository,
            "branch": run.branch,
            "commit_sha": run.commit_sha,
            "started_at": run.started_at.isoformat() if run.started_at else None,
            "completed_at": run.completed_at.isoformat() if run.completed_at else None,
            "duration_seconds": run.duration_seconds,
        }, indent=2))
    else:
        _display_results_table(run)


def _display_results_table(run):
    """Display results in table format."""
    status_color = "green" if run.status == ValidationStatus.PASSED else "red"

    console.print()
    console.print(Panel(
        f"[{status_color}]{run.status.value.upper()}[/{status_color}]",
        title="Validation Result",
    ))

    # Run info
    table = Table(show_header=False, title="Run Details")
    table.add_column("Field", style="cyan")
    table.add_column("Value")

    table.add_row("Run ID", str(run.run_id))
    table.add_row("Repository", run.repository)
    table.add_row("Branch", run.branch)
    table.add_row("Commit", run.commit_sha[:8] if run.commit_sha else "N/A")
    if run.duration_seconds:
        table.add_row("Duration", f"{run.duration_seconds:.2f}s")

    console.print(table)

    # Quality report
    if run.quality_report:
        console.print()
        console.print("[bold]Code Quality[/bold]")
        report = run.quality_report
        q_table = Table(show_header=False)
        q_table.add_column("Metric", style="cyan")
        q_table.add_column("Value")
        q_table.add_row("Status", report.status.value)
        q_table.add_row("Violations", str(report.linting.total_violations))
        q_table.add_row("Coverage", f"{report.coverage.line_coverage_percent:.1f}%")
        q_table.add_row("Quality Gate", "PASSED" if report.summary.quality_gate_passed else "FAILED")
        console.print(q_table)

    # Test results
    if run.test_results:
        console.print()
        console.print("[bold]Test Results[/bold]")
        for result in run.test_results:
            status_color = "green" if result.status.value == "passed" else "red"
            console.print(
                f"  {result.suite_type.value}: "
                f"[{status_color}]{result.status.value}[/{status_color}] "
                f"({result.passed_count}/{result.total_tests} passed)"
            )

    # Deployment checklist
    if run.deployment_checklist:
        console.print()
        console.print("[bold]Deployment Checks[/bold]")
        checklist = run.deployment_checklist
        d_table = Table(show_header=False)
        d_table.add_column("Check", style="cyan")
        d_table.add_column("Status")
        d_table.add_row("Configuration", checklist.configuration.status.value)
        d_table.add_row("Dependencies", checklist.dependencies.status.value)
        d_table.add_row("Compatibility", checklist.compatibility.status.value)
        console.print(d_table)
