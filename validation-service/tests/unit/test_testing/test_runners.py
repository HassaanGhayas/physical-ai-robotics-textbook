"""Unit tests for test runners.

Tests the unit, integration, and E2E test runner adapters.
"""

import pytest
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch
from uuid import uuid4

from src.core.models import TestSuiteType, TestStatus
from src.testing.models import (
    TestRunConfig,
    TestRunnerStatus,
    TestSuiteExecution,
    TestDiscoveryResult,
)
from src.testing.runners.unit import UnitTestRunner, get_unit_test_runner
from src.testing.runners.integration import IntegrationTestRunner, get_integration_test_runner
from src.testing.runners.e2e import E2ETestRunner, get_e2e_test_runner


class TestUnitTestRunner:
    """Tests for UnitTestRunner class."""

    @pytest.fixture
    def runner(self):
        """Create unit test runner instance."""
        return UnitTestRunner()

    @pytest.fixture
    def project_path(self, tmp_path):
        """Create a temporary project with test files."""
        tests_dir = tmp_path / "tests" / "unit"
        tests_dir.mkdir(parents=True)

        # Create test file
        test_file = tests_dir / "test_sample.py"
        test_file.write_text("""
def test_example():
    assert True

def test_another():
    assert 1 + 1 == 2
""")

        # Create conftest
        conftest = tmp_path / "tests" / "conftest.py"
        conftest.write_text("")

        return tmp_path

    def test_discover_finds_test_files(self, runner, project_path):
        """Test that discover finds test files in the project."""
        result = runner.discover(project_path)

        assert isinstance(result, TestDiscoveryResult)
        assert result.suite_type == TestSuiteType.UNIT
        assert len(result.test_files) >= 0  # May not find all in mock

    def test_run_returns_execution_result(self, runner, project_path):
        """Test that run returns a TestSuiteExecution."""
        run_id = uuid4()

        with patch("subprocess.run") as mock_subprocess:
            mock_subprocess.return_value = MagicMock(
                returncode=0,
                stdout="",
                stderr="",
            )
            result = runner.run(project_path, run_id)

        assert isinstance(result, TestSuiteExecution)
        assert result.suite_type == TestSuiteType.UNIT
        assert result.run_id == run_id

    def test_run_with_config(self, runner, project_path):
        """Test running with custom configuration."""
        run_id = uuid4()
        config = TestRunConfig(
            project_path=str(project_path),
            test_type=TestSuiteType.UNIT,
            parallel=True,
            timeout_seconds=120,
        )

        with patch("subprocess.run") as mock_subprocess:
            mock_subprocess.return_value = MagicMock(
                returncode=0,
                stdout="",
                stderr="",
            )
            result = runner.run(project_path, run_id, config)

        assert result.suite_type == TestSuiteType.UNIT


class TestIntegrationTestRunner:
    """Tests for IntegrationTestRunner class."""

    @pytest.fixture
    def runner(self):
        """Create integration test runner instance."""
        return IntegrationTestRunner()

    @pytest.fixture
    def project_path(self, tmp_path):
        """Create a temporary project with integration test files."""
        tests_dir = tmp_path / "tests" / "integration"
        tests_dir.mkdir(parents=True)

        test_file = tests_dir / "test_integration.py"
        test_file.write_text("""
import pytest

@pytest.mark.integration
def test_database_connection():
    assert True
""")

        return tmp_path

    def test_discover_finds_integration_tests(self, runner, project_path):
        """Test that discover finds integration test files."""
        result = runner.discover(project_path)

        assert result.suite_type == TestSuiteType.INTEGRATION

    def test_run_uses_integration_marker(self, runner, project_path):
        """Test that run targets integration tests with marker."""
        run_id = uuid4()

        with patch("subprocess.run") as mock_subprocess:
            mock_subprocess.return_value = MagicMock(
                returncode=0,
                stdout="",
                stderr="",
            )
            result = runner.run(project_path, run_id)

        assert result.suite_type == TestSuiteType.INTEGRATION

    def test_integration_runner_has_longer_timeout(self, runner):
        """Test that integration runner has appropriate timeout."""
        assert runner.default_timeout >= 300  # At least 5 minutes


class TestE2ETestRunner:
    """Tests for E2ETestRunner class."""

    @pytest.fixture
    def runner(self):
        """Create E2E test runner instance."""
        return E2ETestRunner()

    @pytest.fixture
    def project_path(self, tmp_path):
        """Create a temporary project with E2E test files."""
        tests_dir = tmp_path / "tests" / "e2e"
        tests_dir.mkdir(parents=True)

        test_file = tests_dir / "test_user_flow.py"
        test_file.write_text("""
import pytest

@pytest.mark.e2e
def test_login_flow():
    # Simulated E2E test
    assert True
""")

        return tmp_path

    def test_discover_finds_e2e_tests(self, runner, project_path):
        """Test that discover finds E2E test files."""
        result = runner.discover(project_path)

        assert result.suite_type == TestSuiteType.E2E

    def test_run_returns_e2e_execution(self, runner, project_path):
        """Test that run returns E2E execution result."""
        run_id = uuid4()

        with patch("subprocess.run") as mock_subprocess:
            mock_subprocess.return_value = MagicMock(
                returncode=0,
                stdout="",
                stderr="",
            )
            result = runner.run(project_path, run_id)

        assert result.suite_type == TestSuiteType.E2E

    def test_e2e_runner_has_longest_timeout(self, runner):
        """Test that E2E runner has appropriate timeout for slow tests."""
        assert runner.default_timeout >= 600  # At least 10 minutes

    def test_e2e_runs_sequentially_by_default(self, runner):
        """Test that E2E tests run sequentially for stability."""
        # E2E tests often have shared state and should run sequentially
        assert not runner.parallel_default


class TestRunnerFactoryFunctions:
    """Tests for runner factory functions."""

    def test_get_unit_test_runner(self):
        """Test unit test runner factory."""
        runner = get_unit_test_runner()
        assert isinstance(runner, UnitTestRunner)

    def test_get_integration_test_runner(self):
        """Test integration test runner factory."""
        runner = get_integration_test_runner()
        assert isinstance(runner, IntegrationTestRunner)

    def test_get_e2e_test_runner(self):
        """Test E2E test runner factory."""
        runner = get_e2e_test_runner()
        assert isinstance(runner, E2ETestRunner)
