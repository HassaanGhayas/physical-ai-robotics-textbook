"""CLI run command for executing validation."""

import asyncio
from pathlib import Path
from typing import Optional

import typer
from rich.console import Console
from rich.panel import Panel

from src.core.config import get_settings
from src.core.logging import setup_logging
from src.core.models import TriggerSource, ValidationStatus
from src.core.pipeline import PipelineConfig, get_pipeline
from src.core.utils import get_git_info


console = Console()


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
    """Run validation pipeline on a project."""
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

    # Display status
    status_color = "green" if run.status == ValidationStatus.PASSED else "red"
    console.print()
    console.print(Panel(
        f"[{status_color}]{run.status.value.upper()}[/{status_color}]",
        title="Validation Result",
    ))

    if run.duration_seconds:
        console.print(f"Duration: {run.duration_seconds:.2f}s")

    # Exit with appropriate code
    if run.status == ValidationStatus.PASSED:
        raise typer.Exit(0)
    else:
        raise typer.Exit(1)
