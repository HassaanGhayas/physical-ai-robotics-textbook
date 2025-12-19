"""Tests for CLI run command."""

import pytest
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

from typer.testing import CliRunner
import typer

from src.cli.commands.run import run_validation
from src.core.models import ValidationStatus, TriggerSource, ValidationRun


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def mock_pipeline():
    """Mock pipeline for testing."""
    with patch("src.cli.commands.run.get_pipeline") as mock:
        pipeline = MagicMock()
        mock.return_value = pipeline
        yield pipeline


@pytest.fixture
def mock_git_info():
    """Mock git info."""
    with patch("src.cli.commands.run.get_git_info") as mock:
        mock.return_value = {
            "is_git_repo": True,
            "repository": "test-repo",
            "branch": "main",
            "commit_sha": "abc123def456",
        }
        yield mock


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
        duration_seconds=30.5,
    )


# Create a typer app for testing
app = typer.Typer()
app.command()(run_validation)


class TestRunCommand:
    """Tests for run command."""

    def test_run_validation_success(self, runner, mock_pipeline, mock_git_info, sample_run, tmp_path):
        """Test successful validation run."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        result = runner.invoke(app, ["--path", str(tmp_path)])

        assert result.exit_code == 0
        assert "PASSED" in result.output

    def test_run_validation_failed(self, runner, mock_pipeline, mock_git_info, tmp_path):
        """Test failed validation run."""
        failed_run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        mock_pipeline.run = AsyncMock(return_value=failed_run)

        result = runner.invoke(app, ["--path", str(tmp_path)])

        assert result.exit_code == 1
        assert "FAILED" in result.output

    def test_run_validation_quality_only(self, runner, mock_pipeline, mock_git_info, sample_run, tmp_path):
        """Test quality-only validation."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        result = runner.invoke(
            app,
            ["--path", str(tmp_path), "--no-tests", "--no-deployment"]
        )

        assert result.exit_code == 0

    def test_run_validation_tests_only(self, runner, mock_pipeline, mock_git_info, sample_run, tmp_path):
        """Test tests-only validation."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        result = runner.invoke(
            app,
            ["--path", str(tmp_path), "--no-quality", "--no-deployment"]
        )

        assert result.exit_code == 0

    def test_run_validation_verbose(self, runner, mock_pipeline, mock_git_info, sample_run, tmp_path):
        """Test verbose output."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        result = runner.invoke(app, ["--path", str(tmp_path), "--verbose"])

        assert result.exit_code == 0

    def test_run_validation_fail_fast(self, runner, mock_pipeline, mock_git_info, sample_run, tmp_path):
        """Test fail-fast option."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        result = runner.invoke(app, ["--path", str(tmp_path), "--fail-fast"])

        assert result.exit_code == 0


class TestRunCommandGitHandling:
    """Tests for run command git handling."""

    def test_non_git_repo(self, runner, mock_pipeline, tmp_path):
        """Test handling non-git repository."""
        with patch("src.cli.commands.run.get_git_info") as mock:
            mock.return_value = {
                "is_git_repo": False,
                "repository": "",
                "branch": "",
                "commit_sha": "",
            }

            sample_run = ValidationRun(
                run_id=uuid4(),
                status=ValidationStatus.PASSED,
                trigger_source=TriggerSource.MANUAL,
                repository=tmp_path.name,
                branch="unknown",
                commit_sha="unknown",
            )
            mock_pipeline.run = AsyncMock(return_value=sample_run)

            result = runner.invoke(app, ["--path", str(tmp_path)])

            assert "Warning: Not a Git repository" in result.output


class TestRunCommandErrors:
    """Tests for run command error handling."""

    def test_invalid_path(self, runner):
        """Test with non-existent path."""
        result = runner.invoke(app, ["--path", "/nonexistent/path"])

        assert result.exit_code != 0

    def test_file_path_instead_of_directory(self, runner, tmp_path):
        """Test with file instead of directory."""
        test_file = tmp_path / "test.txt"
        test_file.write_text("test")

        result = runner.invoke(app, ["--path", str(test_file)])

        assert result.exit_code != 0

    def test_pipeline_error(self, runner, mock_pipeline, mock_git_info, tmp_path):
        """Test handling pipeline errors."""
        mock_pipeline.run = AsyncMock(side_effect=Exception("Pipeline failed"))

        result = runner.invoke(app, ["--path", str(tmp_path)])

        assert result.exit_code != 0
