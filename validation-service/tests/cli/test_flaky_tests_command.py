"""Tests for CLI flaky-tests command."""

import pytest
from unittest.mock import MagicMock, patch

from typer.testing import CliRunner
import typer

from src.cli.commands.flaky_tests import list_flaky_tests, update_flaky_test
from src.core.models import FlakyTestRecord as FlakyTestInfo, FlakyStatus


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def mock_storage():
    """Mock storage for testing."""
    with patch("src.cli.commands.flaky_tests.get_storage") as mock:
        storage = MagicMock()
        mock.return_value = storage
        yield storage


@pytest.fixture
def sample_flaky_tests():
    """Create sample flaky tests."""
    return [
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


# Create typer app for testing
flaky_app = typer.Typer()
flaky_app.command(name="list")(list_flaky_tests)
flaky_app.command(name="update")(update_flaky_test)


class TestFlakyTestsListCommand:
    """Tests for flaky tests list command."""

    def test_list_flaky_tests_success(self, runner, mock_storage, sample_flaky_tests):
        """Test listing flaky tests."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(flaky_app, ["list"])

        assert result.exit_code == 0
        assert "test_flaky_1" in result.output
        assert "test_flaky_2" in result.output

    def test_list_flaky_tests_empty(self, runner, mock_storage):
        """Test listing when no flaky tests exist."""
        mock_storage.get_active_flaky_tests.return_value = []

        result = runner.invoke(flaky_app, ["list"])

        assert result.exit_code == 0
        assert "No flaky tests" in result.output

    def test_list_flaky_tests_with_status_filter(self, runner, mock_storage, sample_flaky_tests):
        """Test filtering by status."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(flaky_app, ["list", "--status", "active_flaky"])

        assert result.exit_code == 0

    def test_list_flaky_tests_with_limit(self, runner, mock_storage, sample_flaky_tests):
        """Test listing with limit."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(flaky_app, ["list", "--limit", "1"])

        assert result.exit_code == 0


class TestFlakyTestsUpdateCommand:
    """Tests for flaky tests update command."""

    def test_update_flaky_test_success(self, runner, mock_storage, sample_flaky_tests):
        """Test updating flaky test status."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(
            flaky_app,
            ["update", "test_module::test_flaky_1", "--status", "resolved"]
        )

        assert result.exit_code == 0
        assert "Updated" in result.output

    def test_update_flaky_test_not_found(self, runner, mock_storage):
        """Test updating non-existent flaky test."""
        mock_storage.get_active_flaky_tests.return_value = []

        result = runner.invoke(
            flaky_app,
            ["update", "nonexistent::test", "--status", "resolved"]
        )

        assert result.exit_code == 1
        assert "not found" in result.output.lower()

    def test_update_flaky_test_invalid_status(self, runner, mock_storage, sample_flaky_tests):
        """Test updating with invalid status."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(
            flaky_app,
            ["update", "test_module::test_flaky_1", "--status", "invalid"]
        )

        assert result.exit_code == 1
        assert "Invalid status" in result.output


class TestFlakyTestsDisplay:
    """Tests for flaky tests display formatting."""

    def test_display_pass_rate(self, runner, mock_storage, sample_flaky_tests):
        """Test pass rate display."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(flaky_app, ["list"])

        assert "75" in result.output  # Pass rate percentage

    def test_display_confidence(self, runner, mock_storage, sample_flaky_tests):
        """Test confidence display."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        result = runner.invoke(flaky_app, ["list"])

        assert "92" in result.output or "88" in result.output  # Confidence

    def test_display_truncated_test_id(self, runner, mock_storage):
        """Test truncation of long test IDs."""
        long_test = FlakyTestInfo(
            test_id="very_long_module_name::very_long_class_name::very_long_test_function_name_that_exceeds_limit",
            pass_rate=0.75,
            sample_size=20,
            confidence=0.92,
            status=FlakyStatus.ACTIVE_FLAKY,
        )
        mock_storage.get_active_flaky_tests.return_value = [long_test]

        result = runner.invoke(flaky_app, ["list"])

        assert result.exit_code == 0
        assert "..." in result.output  # Truncation indicator
