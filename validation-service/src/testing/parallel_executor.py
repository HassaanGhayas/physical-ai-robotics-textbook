"""Parallel test execution support.

This module provides parallel test execution capabilities using
concurrent.futures for improved performance.
"""

import logging
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import TestSuiteType, TestStatus
from src.testing.models import (
    TestSuiteExecution,
    TestRunnerStatus,
    TestExecutionResult,
    TestRunConfig,
)


logger = logging.getLogger(__name__)


@dataclass
class ParallelExecutionConfig:
    """Configuration for parallel test execution."""

    max_workers: int = 4
    timeout_seconds: int = 300
    fail_fast: bool = False


class ParallelTestExecutor:
    """Service for running tests in parallel."""

    def __init__(self, config: Optional[ParallelExecutionConfig] = None):
        """Initialize the parallel executor.

        Args:
            config: Parallel execution configuration
        """
        self.settings = get_settings()
        self.config = config or ParallelExecutionConfig()

    def execute_tests(
        self,
        test_functions: list[Callable[[], TestExecutionResult]],
        run_id: UUID,
        suite_type: TestSuiteType,
    ) -> TestSuiteExecution:
        """Execute multiple test functions in parallel.

        Args:
            test_functions: List of callable test functions
            run_id: Validation run ID
            suite_type: Type of test suite

        Returns:
            TestSuiteExecution with aggregated results
        """
        from uuid import uuid4

        execution = TestSuiteExecution(
            execution_id=uuid4(),
            run_id=run_id,
            suite_type=suite_type,
            status=TestRunnerStatus.RUNNING,
            started_at=datetime.utcnow(),
        )

        results: list[TestExecutionResult] = []
        failed = False

        with ThreadPoolExecutor(max_workers=self.config.max_workers) as executor:
            # Submit all test functions
            future_to_test = {
                executor.submit(func): i for i, func in enumerate(test_functions)
            }

            for future in as_completed(
                future_to_test, timeout=self.config.timeout_seconds
            ):
                try:
                    result = future.result()
                    results.append(result)

                    # Check fail fast
                    if self.config.fail_fast and result.status == TestStatus.FAILED:
                        failed = True
                        logger.info("Fail fast triggered, cancelling remaining tests")
                        # Cancel remaining futures
                        for f in future_to_test:
                            f.cancel()
                        break

                except TimeoutError:
                    logger.error(
                        f"Test execution timed out after {self.config.timeout_seconds}s"
                    )
                    execution.status = TestRunnerStatus.TIMEOUT
                    break
                except Exception as e:
                    logger.error(f"Test execution error: {e}")
                    # Create error result
                    results.append(
                        TestExecutionResult(
                            test_id=f"error_{future_to_test[future]}",
                            test_name="Unknown Test",
                            status=TestStatus.ERROR,
                            duration_seconds=0.0,
                            error_message=str(e),
                        )
                    )

        # Finalize execution
        execution.completed_at = datetime.utcnow()
        execution.results = results
        execution.total_tests = len(results)
        execution.passed = sum(1 for r in results if r.status == TestStatus.PASSED)
        execution.failed = sum(1 for r in results if r.status == TestStatus.FAILED)
        execution.skipped = sum(1 for r in results if r.status == TestStatus.SKIPPED)
        execution.errors = sum(1 for r in results if r.status == TestStatus.ERROR)

        if execution.status != TestRunnerStatus.TIMEOUT:
            execution.status = (
                TestRunnerStatus.COMPLETED
                if not failed and execution.errors == 0
                else TestRunnerStatus.FAILED
            )

        return execution

    def execute_test_files(
        self,
        test_files: list[Path],
        run_id: UUID,
        suite_type: TestSuiteType,
        runner_func: Callable[[Path], TestExecutionResult],
    ) -> TestSuiteExecution:
        """Execute test files in parallel.

        Args:
            test_files: List of test file paths
            run_id: Validation run ID
            suite_type: Type of test suite
            runner_func: Function to run a single test file

        Returns:
            TestSuiteExecution with aggregated results
        """
        # Create callables for each test file
        test_functions = [lambda f=f: runner_func(f) for f in test_files]

        return self.execute_tests(test_functions, run_id, suite_type)

    def partition_tests(
        self,
        test_files: list[Path],
        num_partitions: int,
    ) -> list[list[Path]]:
        """Partition tests for distributed execution.

        Args:
            test_files: List of test file paths
            num_partitions: Number of partitions

        Returns:
            List of test file partitions
        """
        if num_partitions <= 0:
            return [test_files]

        partitions: list[list[Path]] = [[] for _ in range(num_partitions)]

        for i, test_file in enumerate(test_files):
            partitions[i % num_partitions].append(test_file)

        return [p for p in partitions if p]  # Remove empty partitions


def get_parallel_executor(
    config: Optional[ParallelExecutionConfig] = None,
) -> ParallelTestExecutor:
    """Get parallel test executor instance.

    Args:
        config: Optional configuration

    Returns:
        ParallelTestExecutor instance
    """
    return ParallelTestExecutor(config)
