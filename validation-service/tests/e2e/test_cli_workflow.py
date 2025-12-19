"""End-to-end tests for CLI workflow."""

import pytest
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock, patch
from uuid import uuid4
from datetime import datetime

from typer.testing import CliRunner
import typer

from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
    QualityReport,
    TestResult,
    SuiteType,
    FlakyTestInfo,
    FlakyStatus,
)


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def sample_project(tmp_path):
    """Create a sample project for testing."""
    # Create minimal project structure
    src_dir = tmp_path / "src"
    src_dir.mkdir()
    (src_dir / "__init__.py").write_text("")
    (src_dir / "main.py").write_text("def main(): pass")

    tests_dir = tmp_path / "tests"
    tests_dir.mkdir()
    (tests_dir / "test_main.py").write_text("def test_main(): pass")

    # Git directory
    git_dir = tmp_path / ".git"
    git_dir.mkdir()

    return tmp_path


# Import CLI commands
from src.cli.commands.run import run_validation
from src.cli.commands.results import view_results as show_results
show_latest = show_results
from src.cli.commands.status import show_status
from src.cli.commands.config import show_config, validate_config
from src.cli.commands.flaky_tests import list_flaky_tests

# Create main CLI app for testing
app = typer.Typer()
app.command(name="run")(run_validation)
app.command(name="results")(show_results)
app.command(name="latest")(show_latest)
app.command(name="status")(show_status)
app.command(name="config")(show_config)
app.command(name="config-validate")(validate_config)
app.command(name="flaky-tests")(list_flaky_tests)


class TestCLIValidationWorkflow:
    """End-to-end tests for CLI validation workflow."""

    def test_full_cli_workflow(self, runner, sample_project):
        """Test complete CLI workflow: run -> check status -> view results."""
        run_id = uuid4()

        completed_run = ValidationRun(
            run_id=run_id,
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-project",
            branch="main",
            commit_sha="abc123def456",
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            duration_seconds=45.5,
            quality_report=QualityReport(
                coverage_percentage=87.5,
                passed=True,
                issues=[],
            ),
            test_results=[
                TestResult(
                    suite_type=SuiteType.UNIT,
                    total_tests=50,
                    passed_tests=50,
                    failed_tests=0,
                    skipped_tests=0,
                    duration_seconds=15.0,
                    passed=True,
                    failures=[],
                ),
            ],
        )

        with patch("src.cli.commands.run.get_pipeline") as pipeline_mock, \
             patch("src.cli.commands.run.get_git_info") as git_mock, \
             patch("src.cli.commands.results.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_storage") as status_storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:

            # Configure mocks
            pipeline_mock.return_value.run = AsyncMock(return_value=completed_run)
            git_mock.return_value = {
                "is_git_repo": True,
                "repository": "test-project",
                "branch": "main",
                "commit_sha": "abc123def456",
            }
            storage_mock.return_value.get_run.return_value = completed_run
            storage_mock.return_value.get_latest_run.return_value = completed_run
            status_storage_mock.return_value.get_latest_run.return_value = completed_run
            status_storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            # Step 1: Run validation
            run_result = runner.invoke(app, ["run", "--path", str(sample_project)])
            assert run_result.exit_code == 0
            assert "PASSED" in run_result.output

            # Step 2: Check status
            status_result = runner.invoke(app, ["status"])
            assert status_result.exit_code == 0
            assert "test-project" in status_result.output

            # Step 3: View latest results
            latest_result = runner.invoke(app, ["latest"])
            assert latest_result.exit_code == 0


class TestCLIConfigWorkflow:
    """Tests for CLI configuration workflow."""

    def test_config_workflow(self, runner):
        """Test configuration workflow: show -> validate."""
        with patch("src.cli.commands.config.get_settings") as mock:
            settings = MagicMock()
            settings.environment = "development"
            settings.log_level = "INFO"
            settings.enable_quality_validation = True
            settings.enable_test_execution = True
            settings.enable_deployment_validation = True
            settings.pipeline_timeout_minutes = 15
            settings.quality = MagicMock()
            settings.quality.min_coverage_percentage = 80.0
            settings.quality.max_cyclomatic_complexity = 10
            settings.testing = MagicMock()
            settings.testing.parallel_execution = True
            settings.testing.flaky_detection_enabled = True
            mock.return_value = settings

            # Step 1: Show configuration
            config_result = runner.invoke(app, ["config"])
            assert config_result.exit_code == 0
            assert "development" in config_result.output

            # Step 2: Validate configuration
            validate_result = runner.invoke(app, ["config-validate"])
            assert validate_result.exit_code == 0
            assert "valid" in validate_result.output.lower()


class TestCLIFlakyTestsWorkflow:
    """Tests for CLI flaky tests workflow."""

    def test_flaky_tests_workflow(self, runner):
        """Test flaky tests workflow: list."""
        flaky_tests = [
            FlakyTestInfo(
                test_id="test_module::test_flaky_1",
                pass_rate=0.75,
                sample_size=20,
                confidence=0.92,
                status=FlakyStatus.ACTIVE_FLAKY,
            ),
            FlakyTestInfo(
                test_id="test_module::test_flaky_2",
                pass_rate=0.85,
                sample_size=15,
                confidence=0.88,
                status=FlakyStatus.ACTIVE_FLAKY,
            ),
        ]

        with patch("src.cli.commands.flaky_tests.get_storage") as mock:
            mock.return_value.get_active_flaky_tests.return_value = flaky_tests

            # List flaky tests
            list_result = runner.invoke(app, ["flaky-tests"])
            assert list_result.exit_code == 0
            assert "test_flaky_1" in list_result.output
            assert "test_flaky_2" in list_result.output


class TestCLIErrorHandling:
    """Tests for CLI error handling."""

    def test_invalid_project_path(self, runner):
        """Test handling of invalid project path."""
        result = runner.invoke(app, ["run", "--path", "/nonexistent/path"])
        assert result.exit_code != 0

    def test_validation_failure_exit_code(self, runner, sample_project):
        """Test exit code on validation failure."""
        failed_run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-project",
            branch="main",
            commit_sha="abc123",
        )

        with patch("src.cli.commands.run.get_pipeline") as pipeline_mock, \
             patch("src.cli.commands.run.get_git_info") as git_mock:

            pipeline_mock.return_value.run = AsyncMock(return_value=failed_run)
            git_mock.return_value = {
                "is_git_repo": True,
                "repository": "test-project",
                "branch": "main",
                "commit_sha": "abc123",
            }

            result = runner.invoke(app, ["run", "--path", str(sample_project)])
            assert result.exit_code == 1
            assert "FAILED" in result.output


class TestCLIVerboseOutput:
    """Tests for CLI verbose output."""

    def test_verbose_run(self, runner, sample_project):
        """Test verbose run output."""
        run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-project",
            branch="main",
            commit_sha="abc123",
            duration_seconds=30.0,
        )

        with patch("src.cli.commands.run.get_pipeline") as pipeline_mock, \
             patch("src.cli.commands.run.get_git_info") as git_mock:

            pipeline_mock.return_value.run = AsyncMock(return_value=run)
            git_mock.return_value = {
                "is_git_repo": True,
                "repository": "test-project",
                "branch": "main",
                "commit_sha": "abc123",
            }

            result = runner.invoke(
                app,
                ["run", "--path", str(sample_project), "--verbose"]
            )
            assert result.exit_code == 0
