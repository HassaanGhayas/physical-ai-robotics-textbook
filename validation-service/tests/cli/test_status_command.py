"""Tests for CLI status command."""

import pytest
from datetime import datetime
from unittest.mock import MagicMock, patch
from uuid import uuid4

from typer.testing import CliRunner
import typer

from src.cli.commands.status import show_status
from src.core.models import ValidationStatus, TriggerSource, ValidationRun, FlakyTestRecord as FlakyTestInfo, FlakyStatus


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def sample_run():
    """Create sample validation run."""
    return ValidationRun(
        run_id=uuid4(),
        status=ValidationStatus.PASSED,
        trigger_source=TriggerSource.CI_CD,
        repository="test-repo",
        branch="main",
        commit_sha="abc123",
        started_at=datetime(2024, 1, 1, 10, 0, 0),
        completed_at=datetime(2024, 1, 1, 10, 5, 0),
    )


# Create typer app for testing - wrap show_status as the callback
status_app = typer.Typer()
status_app.callback(invoke_without_command=True)(show_status)


class TestStatusCommand:
    """Tests for status command."""

    def test_show_status_success(self, runner, sample_run):
        """Test showing service status."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = sample_run
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert result.exit_code == 0
            assert "Status" in result.output

    def test_show_status_with_run(self, runner, sample_run):
        """Test status with recent run."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = sample_run
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert "test-repo" in result.output
            assert "main" in result.output

    def test_show_status_no_runs(self, runner):
        """Test status with no recent runs."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert result.exit_code == 0
            assert "No recent runs" in result.output


class TestStatusDisplay:
    """Tests for status display formatting."""

    def test_display_environment(self, runner):
        """Test environment display."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert "development" in result.output

    def test_display_enabled_features(self, runner):
        """Test enabled features display."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert "Enabled" in result.output

    def test_display_disabled_features(self, runner):
        """Test disabled features display."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=False,
                enable_test_execution=False,
                enable_deployment_validation=False,
            )

            result = runner.invoke(status_app, [])

            assert "Disabled" in result.output


class TestStatusFlakyTests:
    """Tests for status flaky tests summary."""

    def test_display_flaky_tests_summary(self, runner):
        """Test flaky tests summary display."""
        flaky_tests = [
            FlakyTestInfo(
                test_id="test_1",
                pass_rate=0.75,
                sample_size=20,
                confidence=0.92,
                status=FlakyStatus.ACTIVE_FLAKY,
            ),
            FlakyTestInfo(
                test_id="test_2",
                pass_rate=0.95,
                sample_size=50,
                confidence=0.95,
                status=FlakyStatus.RESOLVED,
            ),
        ]
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = flaky_tests
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            # Check for flaky test indicators (may show count or list)
            assert result.exit_code == 0

    def test_display_no_flaky_tests(self, runner):
        """Test no flaky tests display."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = None
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert result.exit_code == 0


class TestStatusRunDetails:
    """Tests for status run details."""

    def test_display_passed_run(self, runner, sample_run):
        """Test passed run display."""
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = sample_run
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert "passed" in result.output.lower()

    def test_display_failed_run(self, runner):
        """Test failed run display."""
        failed_run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.CI_CD,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        with patch("src.cli.commands.status.get_storage") as storage_mock, \
             patch("src.cli.commands.status.get_settings") as settings_mock:
            storage_mock.return_value.get_latest_run.return_value = failed_run
            storage_mock.return_value.get_active_flaky_tests.return_value = []
            settings_mock.return_value = MagicMock(
                environment="development",
                enable_quality_validation=True,
                enable_test_execution=True,
                enable_deployment_validation=True,
            )

            result = runner.invoke(status_app, [])

            assert "failed" in result.output.lower()
