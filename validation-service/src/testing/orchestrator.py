"""Test orchestrator service.

This module coordinates test execution across all test types (unit, integration, E2E)
following the proper order: unit → integration → E2E.
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    TestSuiteType,
    TestSuiteResult,
    TestStatus,
    TestCaseResult,
)
from src.core.pipeline import PipelineContext
from src.testing.models import TestSuiteExecution, TestRunnerStatus
from src.testing.runners.unit import UnitTestRunner, get_unit_test_runner
from src.testing.runners.integration import IntegrationTestRunner, get_integration_test_runner
from src.testing.runners.e2e import E2ETestRunner, get_e2e_test_runner


logger = logging.getLogger(__name__)


class TestOrchestrator:
    """Orchestrates test execution across all test types."""

    def __init__(
        self,
        unit_runner: Optional[UnitTestRunner] = None,
        integration_runner: Optional[IntegrationTestRunner] = None,
        e2e_runner: Optional[E2ETestRunner] = None,
    ):
        """Initialize the test orchestrator.

        Args:
            unit_runner: Unit test runner (optional)
            integration_runner: Integration test runner (optional)
            e2e_runner: E2E test runner (optional)
        """
        self.settings = get_settings()
        self.unit_runner = unit_runner or get_unit_test_runner()
        self.integration_runner = integration_runner or get_integration_test_runner()
        self.e2e_runner = e2e_runner or get_e2e_test_runner()

    def run_all(
        self,
        project_path: Path,
        run_id: UUID,
        stop_on_failure: bool = True,
        suite_types: Optional[list[TestSuiteType]] = None,
    ) -> list[TestSuiteResult]:
        """Run all test suites in order.

        Args:
            project_path: Path to project root
            run_id: Validation run ID
            stop_on_failure: Stop execution if a suite fails
            suite_types: Specific suite types to run (default: all)

        Returns:
            List of TestSuiteResult for each suite
        """
        logger.info(f"Starting test orchestration for {project_path}")

        results = []
        suite_types = suite_types or [
            TestSuiteType.UNIT,
            TestSuiteType.INTEGRATION,
            TestSuiteType.E2E,
        ]

        # Execute in order: unit → integration → e2e
        for suite_type in suite_types:
            logger.info(f"Running {suite_type.value} tests")

            try:
                execution = self._run_suite(project_path, run_id, suite_type)
                result = self._convert_to_result(execution, run_id)
                results.append(result)

                # Check if we should stop on failure
                if stop_on_failure and result.failed_count > 0:
                    logger.warning(
                        f"{suite_type.value} tests failed with {result.failed_count} failures. "
                        "Stopping execution."
                    )
                    break

            except Exception as e:
                logger.error(f"Error running {suite_type.value} tests: {e}")
                # Create a failed result for this suite
                result = TestSuiteResult(
                    run_id=run_id,
                    suite_type=suite_type,
                    status=TestStatus.ERROR,
                    started_at=datetime.utcnow(),
                    completed_at=datetime.utcnow(),
                )
                results.append(result)

                if stop_on_failure:
                    break

        return results

    def _run_suite(
        self,
        project_path: Path,
        run_id: UUID,
        suite_type: TestSuiteType,
    ) -> TestSuiteExecution:
        """Run a specific test suite.

        Args:
            project_path: Path to project root
            run_id: Validation run ID
            suite_type: Type of test suite to run

        Returns:
            TestSuiteExecution with results
        """
        runner_map = {
            TestSuiteType.UNIT: self.unit_runner,
            TestSuiteType.INTEGRATION: self.integration_runner,
            TestSuiteType.E2E: self.e2e_runner,
        }

        runner = runner_map.get(suite_type)
        if not runner:
            raise ValueError(f"Unknown suite type: {suite_type}")

        return runner.run(project_path, run_id)

    def _convert_to_result(
        self,
        execution: TestSuiteExecution,
        run_id: UUID,
    ) -> TestSuiteResult:
        """Convert TestSuiteExecution to TestSuiteResult.

        Args:
            execution: Execution details
            run_id: Validation run ID

        Returns:
            TestSuiteResult for storage/API
        """
        # Determine overall status
        if execution.status == TestRunnerStatus.TIMEOUT:
            status = TestStatus.ERROR
        elif execution.failed > 0 or execution.errors > 0:
            status = TestStatus.FAILED
        elif execution.passed > 0:
            status = TestStatus.PASSED
        elif execution.skipped == execution.total_tests:
            status = TestStatus.SKIPPED
        else:
            status = TestStatus.PASSED  # No tests = passed

        # Convert individual results
        test_cases = []
        for result in execution.results:
            test_case = TestCaseResult(
                test_id=result.test_id,
                test_name=result.test_name,
                status=result.status,
                duration_seconds=result.duration_seconds,
                error_message=result.error_message,
                stack_trace=result.stack_trace,
                retries=result.retry_count,
            )
            test_cases.append(test_case)

        return TestSuiteResult(
            run_id=run_id,
            suite_type=execution.suite_type,
            status=status,
            started_at=execution.started_at or datetime.utcnow(),
            completed_at=execution.completed_at or datetime.utcnow(),
            duration_seconds=(
                (execution.completed_at - execution.started_at).total_seconds()
                if execution.started_at and execution.completed_at
                else 0.0
            ),
            total_tests=execution.total_tests,
            passed_count=execution.passed,
            failed_count=execution.failed,
            skipped_count=execution.skipped,
            error_count=execution.errors,
            test_cases=test_cases,
            flaky_tests_detected=execution.flaky_tests,
            coverage_contribution=execution.coverage_percent,
        )

    def discover_tests(self, project_path: Path) -> dict[TestSuiteType, int]:
        """Discover all tests in the project.

        Args:
            project_path: Path to project root

        Returns:
            Dict mapping suite type to test count
        """
        counts = {}

        for suite_type, runner in [
            (TestSuiteType.UNIT, self.unit_runner),
            (TestSuiteType.INTEGRATION, self.integration_runner),
            (TestSuiteType.E2E, self.e2e_runner),
        ]:
            discovery = runner.discover(project_path)
            counts[suite_type] = discovery.total_tests

        return counts


async def testing_stage_handler(context: PipelineContext) -> dict:
    """Pipeline stage handler for test execution.

    This function is registered with the pipeline orchestrator to handle
    the test execution stage.

    Args:
        context: Pipeline execution context

    Returns:
        Dictionary with status and results
    """
    orchestrator = get_test_orchestrator()

    results = orchestrator.run_all(
        project_path=Path(context.project_path),
        run_id=context.run.run_id,
        stop_on_failure=True,
    )

    # Attach results to the run
    context.run.test_results = results

    # Determine overall status
    all_passed = all(r.status == TestStatus.PASSED for r in results)

    return {
        "status": "passed" if all_passed else "failed",
        "results": results,
    }


def get_test_orchestrator() -> TestOrchestrator:
    """Get test orchestrator instance.

    Returns:
        TestOrchestrator instance
    """
    return TestOrchestrator()
