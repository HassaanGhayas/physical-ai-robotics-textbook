"""Unit tests for test orchestrator service.

Tests the TestOrchestrator class and its coordination of test execution.
"""

import pytest
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock
from uuid import uuid4

from src.core.models import TestSuiteType, TestStatus
from src.testing.orchestrator import TestOrchestrator, get_test_orchestrator
from src.testing.models import (
    TestSuiteExecution,
    TestRunnerStatus,
    TestExecutionResult,
)


class TestTestOrchestrator:
    """Tests for TestOrchestrator class."""

    @pytest.fixture
    def mock_unit_runner(self):
        """Create mock unit test runner."""
        runner = MagicMock()
        runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=5,
            passed=5,
            failed=0,
            skipped=0,
            errors=0,
        )
        return runner

    @pytest.fixture
    def mock_integration_runner(self):
        """Create mock integration test runner."""
        runner = MagicMock()
        runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.INTEGRATION,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=3,
            passed=3,
            failed=0,
            skipped=0,
            errors=0,
        )
        return runner

    @pytest.fixture
    def mock_e2e_runner(self):
        """Create mock E2E test runner."""
        runner = MagicMock()
        runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.E2E,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=2,
            passed=2,
            failed=0,
            skipped=0,
            errors=0,
        )
        return runner

    @pytest.fixture
    def orchestrator(self, mock_unit_runner, mock_integration_runner, mock_e2e_runner):
        """Create orchestrator with mock runners."""
        return TestOrchestrator(
            unit_runner=mock_unit_runner,
            integration_runner=mock_integration_runner,
            e2e_runner=mock_e2e_runner,
        )

    def test_run_all_passes_when_all_suites_pass(
        self, orchestrator, mock_unit_runner, mock_integration_runner, mock_e2e_runner
    ):
        """Test that run_all returns passing results when all suites pass."""
        project_path = Path("/test/project")
        run_id = uuid4()

        results = orchestrator.run_all(project_path, run_id)

        assert len(results) == 3
        assert all(r.status == TestStatus.PASSED for r in results)
        mock_unit_runner.run.assert_called_once()
        mock_integration_runner.run.assert_called_once()
        mock_e2e_runner.run.assert_called_once()

    def test_run_all_stops_on_failure_when_stop_on_failure_true(
        self, orchestrator, mock_unit_runner, mock_integration_runner
    ):
        """Test that run_all stops after first failure when stop_on_failure=True."""
        # Make unit tests fail
        mock_unit_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.FAILED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=5,
            passed=3,
            failed=2,
            skipped=0,
            errors=0,
        )

        project_path = Path("/test/project")
        run_id = uuid4()

        results = orchestrator.run_all(project_path, run_id, stop_on_failure=True)

        # Should stop after unit tests
        assert len(results) == 1
        assert results[0].status == TestStatus.FAILED
        mock_unit_runner.run.assert_called_once()
        mock_integration_runner.run.assert_not_called()

    def test_run_all_continues_on_failure_when_stop_on_failure_false(
        self, orchestrator, mock_unit_runner, mock_integration_runner, mock_e2e_runner
    ):
        """Test that run_all continues after failure when stop_on_failure=False."""
        # Make unit tests fail
        mock_unit_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.FAILED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=5,
            passed=3,
            failed=2,
            skipped=0,
            errors=0,
        )

        project_path = Path("/test/project")
        run_id = uuid4()

        results = orchestrator.run_all(project_path, run_id, stop_on_failure=False)

        # Should continue running all suites
        assert len(results) == 3
        mock_unit_runner.run.assert_called_once()
        mock_integration_runner.run.assert_called_once()
        mock_e2e_runner.run.assert_called_once()

    def test_run_specific_suite_types(
        self, orchestrator, mock_unit_runner, mock_integration_runner, mock_e2e_runner
    ):
        """Test running only specific suite types."""
        project_path = Path("/test/project")
        run_id = uuid4()

        # Only run unit and integration tests
        results = orchestrator.run_all(
            project_path,
            run_id,
            suite_types=[TestSuiteType.UNIT, TestSuiteType.INTEGRATION],
        )

        assert len(results) == 2
        mock_unit_runner.run.assert_called_once()
        mock_integration_runner.run.assert_called_once()
        mock_e2e_runner.run.assert_not_called()

    def test_run_handles_runner_exception(self, orchestrator, mock_unit_runner):
        """Test that run_all handles exceptions from runners."""
        mock_unit_runner.run.side_effect = Exception("Runner crashed")

        project_path = Path("/test/project")
        run_id = uuid4()

        results = orchestrator.run_all(
            project_path, run_id, stop_on_failure=True
        )

        assert len(results) == 1
        assert results[0].status == TestStatus.ERROR

    def test_discover_tests(
        self, orchestrator, mock_unit_runner, mock_integration_runner, mock_e2e_runner
    ):
        """Test test discovery across all suite types."""
        from src.testing.models import TestDiscoveryResult

        mock_unit_runner.discover.return_value = TestDiscoveryResult(
            suite_type=TestSuiteType.UNIT,
            total_tests=10,
            test_files=["test_a.py", "test_b.py"],
        )
        mock_integration_runner.discover.return_value = TestDiscoveryResult(
            suite_type=TestSuiteType.INTEGRATION,
            total_tests=5,
            test_files=["test_integration.py"],
        )
        mock_e2e_runner.discover.return_value = TestDiscoveryResult(
            suite_type=TestSuiteType.E2E,
            total_tests=3,
            test_files=["test_e2e.py"],
        )

        project_path = Path("/test/project")
        counts = orchestrator.discover_tests(project_path)

        assert counts[TestSuiteType.UNIT] == 10
        assert counts[TestSuiteType.INTEGRATION] == 5
        assert counts[TestSuiteType.E2E] == 3


class TestGetTestOrchestrator:
    """Tests for get_test_orchestrator factory function."""

    def test_returns_orchestrator_instance(self):
        """Test that get_test_orchestrator returns a TestOrchestrator."""
        with patch("src.testing.orchestrator.get_unit_test_runner"), \
             patch("src.testing.orchestrator.get_integration_test_runner"), \
             patch("src.testing.orchestrator.get_e2e_test_runner"):
            orchestrator = get_test_orchestrator()
            assert isinstance(orchestrator, TestOrchestrator)
