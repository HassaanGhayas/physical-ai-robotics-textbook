"""CLI interface for the validation service.

This module provides the command-line interface for running validation,
viewing results, and managing flaky tests.
"""

import asyncio
import sys
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel
from rich.table import Table

from src.core.config import get_settings
from src.core.logging import setup_logging
from src.core.models import TriggerSource, ValidationStatus
from src.core.pipeline import PipelineConfig, get_pipeline
from src.core.storage import get_storage
from src.core.utils import get_git_info
from src.quality.validator import get_quality_validator


app = typer.Typer(
    name="validate",
    help="Validation Service CLI - Code quality, testing, and deployment validation",
    add_completion=False,
)
console = Console()


@app.command("run")
def run_validation(
    path: Path = typer.Option(
        Path("."),
        "--path", "-p",
        help="Path to project to validate",
        exists=True,
        file_okay=False,
        resolve_path=True,
    ),
    quality: bool = typer.Option(True, "--quality/--no-quality", help="Run quality checks"),
    tests: bool = typer.Option(True, "--tests/--no-tests", help="Run test suites"),
    deployment: bool = typer.Option(True, "--deployment/--no-deployment", help="Run deployment checks"),
    fail_fast: bool = typer.Option(False, "--fail-fast", help="Stop on first failure"),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Verbose output"),
):
    """Run the full validation pipeline."""
    setup_logging(level="DEBUG" if verbose else "INFO")

    console.print(Panel.fit(
        "[bold blue]Validation Service[/bold blue]\n"
        f"Project: {path}",
        title="Starting Validation",
    ))

    # Get Git info
    git_info = get_git_info(path)
    if not git_info["is_git_repo"]:
        console.print("[yellow]Warning: Not a Git repository[/yellow]")
        git_info["repository"] = path.name
        git_info["branch"] = "unknown"
        git_info["commit_sha"] = "unknown"

    console.print(f"Repository: {git_info['repository']}")
    console.print(f"Branch: {git_info['branch']}")
    console.print(f"Commit: {git_info['commit_sha'][:8] if len(git_info['commit_sha']) > 8 else git_info['commit_sha']}")
    console.print()

    # Create pipeline config
    config = PipelineConfig(
        run_quality=quality,
        run_tests=tests,
        run_deployment=deployment,
        fail_fast=fail_fast,
    )

    # Run pipeline
    pipeline = get_pipeline()
    run = asyncio.run(
        pipeline.run(
            project_path=str(path),
            repository=git_info["repository"],
            branch=git_info["branch"],
            commit_sha=git_info["commit_sha"],
            trigger_source=TriggerSource.MANUAL,
            config=config,
        )
    )

    # Display results
    _display_results(run)

    # Exit with appropriate code
    if run.status == ValidationStatus.PASSED:
        raise typer.Exit(0)
    else:
        raise typer.Exit(1)


@app.command("quality")
def run_quality_only(
    path: Path = typer.Option(
        Path("."),
        "--path", "-p",
        help="Path to project to validate",
        exists=True,
        file_okay=False,
        resolve_path=True,
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Verbose output"),
):
    """Run only code quality validation."""
    setup_logging(level="DEBUG" if verbose else "INFO")

    console.print(Panel.fit(
        "[bold blue]Quality Validation[/bold blue]\n"
        f"Project: {path}",
        title="Starting Quality Checks",
    ))

    # Get Git info
    git_info = get_git_info(path)
    if not git_info["is_git_repo"]:
        git_info["repository"] = path.name
        git_info["branch"] = "unknown"
        git_info["commit_sha"] = "unknown"

    # Run quality only
    pipeline = get_pipeline()
    run = asyncio.run(
        pipeline.run_quality_only(
            project_path=str(path),
            repository=git_info["repository"],
            branch=git_info["branch"],
            commit_sha=git_info["commit_sha"],
        )
    )

    _display_quality_results(run)

    if run.status == ValidationStatus.PASSED:
        raise typer.Exit(0)
    else:
        raise typer.Exit(1)


@app.command("tests")
def run_tests_only(
    path: Path = typer.Option(
        Path("."),
        "--path", "-p",
        help="Path to project to validate",
        exists=True,
        file_okay=False,
        resolve_path=True,
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Verbose output"),
):
    """Run only test suites."""
    setup_logging(level="DEBUG" if verbose else "INFO")

    console.print(Panel.fit(
        "[bold blue]Test Execution[/bold blue]\n"
        f"Project: {path}",
        title="Running Tests",
    ))

    git_info = get_git_info(path)
    if not git_info["is_git_repo"]:
        git_info["repository"] = path.name
        git_info["branch"] = "unknown"
        git_info["commit_sha"] = "unknown"

    pipeline = get_pipeline()
    run = asyncio.run(
        pipeline.run_tests_only(
            project_path=str(path),
            repository=git_info["repository"],
            branch=git_info["branch"],
            commit_sha=git_info["commit_sha"],
        )
    )

    _display_test_results(run)

    if run.status == ValidationStatus.PASSED:
        raise typer.Exit(0)
    else:
        raise typer.Exit(1)


@app.command("deployment")
def run_deployment_only(
    path: Path = typer.Option(
        Path("."),
        "--path", "-p",
        help="Path to project to validate",
        exists=True,
        file_okay=False,
        resolve_path=True,
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Verbose output"),
):
    """Run only deployment validation."""
    setup_logging(level="DEBUG" if verbose else "INFO")

    console.print(Panel.fit(
        "[bold blue]Deployment Validation[/bold blue]\n"
        f"Project: {path}",
        title="Running Deployment Checks",
    ))

    git_info = get_git_info(path)
    if not git_info["is_git_repo"]:
        git_info["repository"] = path.name
        git_info["branch"] = "unknown"
        git_info["commit_sha"] = "unknown"

    pipeline = get_pipeline()
    run = asyncio.run(
        pipeline.run_deployment_only(
            project_path=str(path),
            repository=git_info["repository"],
            branch=git_info["branch"],
            commit_sha=git_info["commit_sha"],
        )
    )

    _display_deployment_results(run)

    if run.status == ValidationStatus.PASSED:
        raise typer.Exit(0)
    else:
        raise typer.Exit(1)


@app.command("results")
def view_results(
    run_id: Optional[str] = typer.Argument(None, help="Run ID to view (latest if omitted)"),
):
    """View validation results."""
    storage = get_storage()

    if run_id:
        from uuid import UUID
        run = storage.get_run(UUID(run_id))
        if not run:
            console.print(f"[red]Run not found: {run_id}[/red]")
            raise typer.Exit(1)
    else:
        run = storage.get_latest_run()
        if not run:
            console.print("[yellow]No validation runs found[/yellow]")
            raise typer.Exit(0)

    _display_results(run)


@app.command("flaky-tests")
def list_flaky_tests():
    """List detected flaky tests."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    if not flaky_tests:
        console.print("[green]No flaky tests detected[/green]")
        raise typer.Exit(0)

    table = Table(title="Flaky Tests")
    table.add_column("Test ID", style="cyan")
    table.add_column("Pass Rate", justify="right")
    table.add_column("Sample Size", justify="right")
    table.add_column("Confidence", justify="right")
    table.add_column("Status")

    for test in flaky_tests:
        status_color = "yellow" if test.status.value == "active_flaky" else "green"
        table.add_row(
            test.test_id[:50] + "..." if len(test.test_id) > 50 else test.test_id,
            f"{test.pass_rate * 100:.1f}%",
            str(test.sample_size),
            f"{test.confidence * 100:.1f}%",
            f"[{status_color}]{test.status.value}[/{status_color}]",
        )

    console.print(table)


@app.command("config")
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


def _display_results(run):
    """Display validation run results."""
    status_color = "green" if run.status == ValidationStatus.PASSED else "red"

    console.print()
    console.print(Panel(
        f"[{status_color}]{run.status.value.upper()}[/{status_color}]",
        title="Validation Result",
    ))

    if run.duration_seconds:
        console.print(f"Duration: {run.duration_seconds:.2f}s")

    _display_quality_results(run)
    _display_test_results(run)
    _display_deployment_results(run)


def _display_quality_results(run):
    """Display quality validation results."""
    if not run.quality_report:
        return

    report = run.quality_report
    console.print()
    console.print("[bold]Code Quality[/bold]")

    table = Table(show_header=False)
    table.add_column("Metric", style="cyan")
    table.add_column("Value")

    table.add_row("Status", report.status.value)
    table.add_row("Linting Violations", str(report.linting.total_violations))
    table.add_row("Linting Errors", str(report.linting.error_count))
    table.add_row("Max Complexity", str(report.complexity.max_complexity))
    table.add_row("Line Coverage", f"{report.coverage.line_coverage_percent:.1f}%")
    table.add_row("Security Issues", str(
        report.security.critical_count +
        report.security.high_count +
        report.security.medium_count +
        report.security.low_count
    ))
    table.add_row("Quality Gate", "PASSED" if report.summary.quality_gate_passed else "FAILED")

    console.print(table)


def _display_test_results(run):
    """Display test execution results."""
    if not run.test_results:
        return

    console.print()
    console.print("[bold]Test Results[/bold]")

    for result in run.test_results:
        status_color = "green" if result.status.value == "passed" else "red"
        console.print(
            f"  {result.suite_type.value}: "
            f"[{status_color}]{result.status.value}[/{status_color}] "
            f"({result.passed_count}/{result.total_tests} passed, "
            f"{result.duration_seconds:.2f}s)"
        )


def _display_deployment_results(run):
    """Display deployment validation results."""
    if not run.deployment_checklist:
        return

    checklist = run.deployment_checklist
    console.print()
    console.print("[bold]Deployment Checks[/bold]")

    status_color = "green" if checklist.status.value == "passed" else "red"
    console.print(f"  Status: [{status_color}]{checklist.status.value}[/{status_color}]")
    console.print(f"  Configuration: {checklist.configuration.status.value}")
    console.print(f"  Dependencies: {checklist.dependencies.status.value}")
    console.print(f"  Compatibility: {checklist.compatibility.status.value}")


if __name__ == "__main__":
    app()
