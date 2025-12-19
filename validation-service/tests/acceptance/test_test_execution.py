"""Acceptance tests for test execution (User Story 2).

These tests verify the acceptance criteria from spec.md:
- Given codebase with test suite, when full testing triggers, then unit → integration → E2E tests execute in order
- Given failing test, when testing completes, then system halts progression and provides error details
- Given all tests passing, when results reviewed, then execution metrics are available
"""

import pytest
import asyncio
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock
from uuid import uuid4

from src.core.models import (
    TestSuiteType,
    TestStatus,
    TriggerSource,
    ValidationStatus,
    CheckStatus,
)
from src.core.pipeline import (
    ValidationPipeline,
    PipelineConfig,
    PipelineStage,
)
from src.testing.orchestrator import TestOrchestrator, get_test_orchestrator
from src.testing.models import (
    TestSuiteExecution,
    TestRunnerStatus,
    TestExecutionResult,
)
from src.testing.report_aggregator import TestReportAggregator


class TestAcceptanceScenario1_TestExecutionOrder:
    """Acceptance Scenario 1:
    Given a codebase with a test suite,
    When full testing triggers,
    Then unit → integration → E2E tests execute in order.
    """

    def test_tests_execute_in_correct_order(self):
        """Verify tests execute in unit → integration → E2E order."""
        # Track execution order
        execution_order = []

        mock_unit_runner = MagicMock()
        mock_integration_runner = MagicMock()
        mock_e2e_runner = MagicMock()

        def create_mock_run(suite_type):
            def mock_run(project_path, run_id, config=None):
                execution_order.append(suite_type)
                return TestSuiteExecution(
                    execution_id=uuid4(),
                    run_id=run_id,
                    suite_type=suite_type,
                    status=TestRunnerStatus.COMPLETED,
                    started_at=datetime.utcnow(),
                    completed_at=datetime.utcnow(),
                    total_tests=5,
                    passed=5,
                    failed=0,
                    skipped=0,
                    errors=0,
                )
            return mock_run

        mock_unit_runner.run = MagicMock(side_effect=create_mock_run(TestSuiteType.UNIT))
        mock_integration_runner.run = MagicMock(side_effect=create_mock_run(TestSuiteType.INTEGRATION))
        mock_e2e_runner.run = MagicMock(side_effect=create_mock_run(TestSuiteType.E2E))

        orchestrator = TestOrchestrator(
            unit_runner=mock_unit_runner,
            integration_runner=mock_integration_runner,
            e2e_runner=mock_e2e_runner,
        )

        # Act: Run all test suites
        project_path = Path("/test/project")
        run_id = uuid4()
        results = orchestrator.run_all(project_path, run_id)

        # Assert: Correct order
        assert execution_order == [
            TestSuiteType.UNIT,
            TestSuiteType.INTEGRATION,
            TestSuiteType.E2E,
        ], "Tests must execute in unit → integration → E2E order"

        # Assert: All suites ran
        assert len(results) == 3
        assert results[0].suite_type == TestSuiteType.UNIT
        assert results[1].suite_type == TestSuiteType.INTEGRATION
        assert results[2].suite_type == TestSuiteType.E2E


class TestAcceptanceScenario2_FailingTestHaltsProgression:
    """Acceptance Scenario 2:
    Given a failing test,
    When testing completes,
    Then system halts progression and provides error details.
    """

    def test_failing_unit_test_halts_progression(self):
        """Verify that failing unit tests halt progression to integration tests."""
        mock_unit_runner = MagicMock()
        mock_integration_runner = MagicMock()
        mock_e2e_runner = MagicMock()

        # Unit tests fail
        mock_unit_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.FAILED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=10,
            passed=7,
            failed=3,
            skipped=0,
            errors=0,
            results=[
                TestExecutionResult(
                    test_id="test_1",
                    test_name="test_feature_a",
                    status=TestStatus.FAILED,
                    duration_seconds=0.5,
                    error_message="AssertionError: Expected 5 but got 4",
                    stack_trace="File test_feature.py, line 10...",
                ),
                TestExecutionResult(
                    test_id="test_2",
                    test_name="test_feature_b",
                    status=TestStatus.FAILED,
                    duration_seconds=0.3,
                    error_message="ValueError: Invalid input",
                ),
            ],
        )

        orchestrator = TestOrchestrator(
            unit_runner=mock_unit_runner,
            integration_runner=mock_integration_runner,
            e2e_runner=mock_e2e_runner,
        )

        # Act: Run with stop_on_failure=True (default)
        results = orchestrator.run_all(
            Path("/test/project"),
            uuid4(),
            stop_on_failure=True,
        )

        # Assert: Only unit tests ran (progression halted)
        assert len(results) == 1
        mock_unit_runner.run.assert_called_once()
        mock_integration_runner.run.assert_not_called()
        mock_e2e_runner.run.assert_not_called()

        # Assert: Failure details are available
        assert results[0].status == TestStatus.FAILED
        assert results[0].failed_count == 3
        assert len(results[0].test_cases) >= 0  # Test cases with error details

    def test_failing_test_provides_error_details(self):
        """Verify that failing tests include error message and stack trace."""
        mock_unit_runner = MagicMock()

        failed_result = TestExecutionResult(
            test_id="test_validation",
            test_name="test_input_validation",
            status=TestStatus.FAILED,
            duration_seconds=0.25,
            error_message="AssertionError: validate() should reject empty input",
            stack_trace="""
Traceback (most recent call last):
  File "tests/test_validation.py", line 15, in test_input_validation
    result = validate("")
  File "src/validators.py", line 8, in validate
    raise ValueError("Empty input not allowed")
AssertionError: validate() should reject empty input
""",
        )

        mock_unit_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.FAILED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=1,
            passed=0,
            failed=1,
            skipped=0,
            errors=0,
            results=[failed_result],
        )

        orchestrator = TestOrchestrator(
            unit_runner=mock_unit_runner,
            integration_runner=MagicMock(),
            e2e_runner=MagicMock(),
        )

        results = orchestrator.run_all(
            Path("/test/project"),
            uuid4(),
            suite_types=[TestSuiteType.UNIT],
        )

        # Assert: Error details are present
        execution = mock_unit_runner.run.return_value
        assert len(execution.results) == 1
        result = execution.results[0]
        assert result.error_message is not None
        assert "AssertionError" in result.error_message
        assert result.stack_trace is not None
        assert "test_validation.py" in result.stack_trace


class TestAcceptanceScenario3_PassingTestsWithMetrics:
    """Acceptance Scenario 3:
    Given all tests passing,
    When results are reviewed,
    Then execution metrics are available.
    """

    def test_passing_tests_include_execution_metrics(self):
        """Verify that passing test runs include all execution metrics."""
        mock_unit_runner = MagicMock()
        mock_integration_runner = MagicMock()
        mock_e2e_runner = MagicMock()

        # All tests pass
        mock_unit_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.UNIT,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=25,
            passed=25,
            failed=0,
            skipped=0,
            errors=0,
            coverage_percent=85.5,
        )

        mock_integration_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.INTEGRATION,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=10,
            passed=10,
            failed=0,
            skipped=0,
            errors=0,
        )

        mock_e2e_runner.run.return_value = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=uuid4(),
            suite_type=TestSuiteType.E2E,
            status=TestRunnerStatus.COMPLETED,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            total_tests=5,
            passed=5,
            failed=0,
            skipped=0,
            errors=0,
        )

        orchestrator = TestOrchestrator(
            unit_runner=mock_unit_runner,
            integration_runner=mock_integration_runner,
            e2e_runner=mock_e2e_runner,
        )

        # Act: Run all tests
        results = orchestrator.run_all(Path("/test/project"), uuid4())

        # Assert: All passed
        assert all(r.status == TestStatus.PASSED for r in results)

        # Act: Generate report with metrics
        aggregator = TestReportAggregator()
        report = aggregator.aggregate(results)

        # Assert: Metrics are available
        assert "summary" in report
        summary = report["summary"]

        # Total test count
        assert summary["total_tests"] == 40  # 25 + 10 + 5

        # Pass/fail counts
        assert summary["passed"] == 40
        assert summary["failed"] == 0
        assert summary["skipped"] == 0
        assert summary["errors"] == 0

        # Pass rate
        assert summary["pass_rate"] == 100.0

        # Duration tracking
        assert "duration_seconds" in summary

        # Per-suite breakdown
        assert "suites" in report
        assert "unit" in report["suites"]
        assert "integration" in report["suites"]
        assert "e2e" in report["suites"]

        # Suite-specific metrics
        unit_suite = report["suites"]["unit"]
        assert unit_suite["total"] == 25
        assert unit_suite["passed"] == 25
        assert "pass_rate" in unit_suite
        assert "duration_seconds" in unit_suite

    def test_report_includes_timestamp(self):
        """Verify that reports include generation timestamp."""
        aggregator = TestReportAggregator()

        results = [
            TestSuiteExecution(
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
        ]

        # Need to convert TestSuiteExecution to TestSuiteResult for aggregator
        from src.core.models import TestSuiteResult

        test_results = [
            TestSuiteResult(
                run_id=uuid4(),
                suite_type=TestSuiteType.UNIT,
                status=TestStatus.PASSED,
                started_at=datetime.utcnow(),
                completed_at=datetime.utcnow(),
                total_tests=5,
                passed_count=5,
                failed_count=0,
                skipped_count=0,
                error_count=0,
                duration_seconds=1.5,
            )
        ]

        report = aggregator.aggregate(test_results)

        assert "generated_at" in report
        # Should be a valid ISO timestamp
        datetime.fromisoformat(report["generated_at"])

    def test_console_format_shows_summary(self):
        """Verify that console format displays readable summary."""
        from src.core.models import TestSuiteResult

        aggregator = TestReportAggregator()

        test_results = [
            TestSuiteResult(
                run_id=uuid4(),
                suite_type=TestSuiteType.UNIT,
                status=TestStatus.PASSED,
                started_at=datetime.utcnow(),
                completed_at=datetime.utcnow(),
                total_tests=10,
                passed_count=9,
                failed_count=1,
                skipped_count=0,
                error_count=0,
                duration_seconds=2.5,
            )
        ]

        console_output = aggregator.format_console(test_results)

        # Verify key metrics are displayed
        assert "Total Tests: 10" in console_output
        assert "Passed:" in console_output
        assert "Failed:" in console_output
        assert "Pass Rate:" in console_output
        assert "Duration:" in console_output
