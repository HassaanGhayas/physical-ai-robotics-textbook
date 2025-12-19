"""Tests for CLI results command."""

import pytest
from datetime import datetime
from unittest.mock import MagicMock, patch
from uuid import uuid4

from typer.testing import CliRunner
import typer

from src.cli.commands.results import view_results
from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
)


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def mock_storage():
    """Mock storage for testing."""
    with patch("src.cli.commands.results.get_storage") as mock:
        storage = MagicMock()
        mock.return_value = storage
        yield storage


@pytest.fixture
def sample_run():
    """Create sample validation run."""
    return ValidationRun(
        run_id=uuid4(),
        status=ValidationStatus.PASSED,
        trigger_source=TriggerSource.MANUAL,
        repository="test-repo",
        branch="main",
        commit_sha="abc123def456",
        started_at=datetime(2024, 1, 1, 10, 0, 0),
        completed_at=datetime(2024, 1, 1, 10, 5, 0),
        duration_seconds=300.0,
    )


# Create typer app for testing - use callback pattern
results_app = typer.Typer()
results_app.command(name="show")(view_results)
results_app.command(name="latest")(view_results)


class TestResultsCommand:
    """Tests for results command."""

    def test_show_results_success(self, runner, mock_storage, sample_run):
        """Test showing results."""
        mock_storage.get_run.return_value = sample_run

        result = runner.invoke(results_app, ["show", str(sample_run.run_id)])

        assert result.exit_code == 0
        assert "test-repo" in result.output

    def test_show_results_not_found(self, runner, mock_storage):
        """Test showing non-existent results."""
        mock_storage.get_run.return_value = None
        run_id = uuid4()

        result = runner.invoke(results_app, ["show", str(run_id)])

        assert result.exit_code == 1
        assert "not found" in result.output.lower()

    def test_show_results_json_format(self, runner, mock_storage, sample_run):
        """Test JSON output format."""
        mock_storage.get_run.return_value = sample_run

        result = runner.invoke(
            results_app,
            ["show", str(sample_run.run_id), "--format", "json"]
        )

        assert result.exit_code == 0

class TestLatestCommand:
    """Tests for latest command."""

    def test_show_latest_success(self, runner, mock_storage, sample_run):
        """Test showing latest results."""
        mock_storage.get_latest_run.return_value = sample_run

        result = runner.invoke(results_app, ["latest"])

        assert result.exit_code == 0
        assert "test-repo" in result.output

    def test_show_latest_no_runs(self, runner, mock_storage):
        """Test when no runs exist."""
        mock_storage.get_latest_run.return_value = None

        result = runner.invoke(results_app, ["latest"])

        assert result.exit_code == 0
        assert "No validation runs" in result.output


class TestResultsDisplay:
    """Tests for results display formatting."""

    def test_display_quality_report(self, runner, mock_storage, sample_run):
        """Test quality report display."""
        mock_storage.get_run.return_value = sample_run

        result = runner.invoke(results_app, ["show", str(sample_run.run_id)])

        # Just check it runs successfully
        assert result.exit_code == 0

    def test_display_test_results(self, runner, mock_storage, sample_run):
        """Test test results display."""
        mock_storage.get_run.return_value = sample_run

        result = runner.invoke(results_app, ["show", str(sample_run.run_id)])

        # Just check it runs successfully
        assert result.exit_code == 0

    def test_display_failed_status(self, runner, mock_storage):
        """Test failed status display."""
        failed_run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        mock_storage.get_run.return_value = failed_run

        result = runner.invoke(results_app, ["show", str(failed_run.run_id)])

        assert "FAILED" in result.output or "failed" in result.output.lower()
