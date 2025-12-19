"""Integration test runner adapter.

This module provides the integration test runner using pytest with integration markers.
"""

import json
import logging
import subprocess
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.exceptions import TestExecutionError
from src.core.models import TestStatus, TestSuiteType
from src.testing.models import (
    TestRunConfig,
    TestExecutionResult,
    TestSuiteExecution,
    TestRunnerStatus,
    TestDiscoveryResult,
)


logger = logging.getLogger(__name__)


class IntegrationTestRunner:
    """Runner for integration tests using pytest."""

    def __init__(self):
        """Initialize the integration test runner."""
        self.settings = get_settings()
        self.suite_type = TestSuiteType.INTEGRATION

    def discover(self, project_path: Path) -> TestDiscoveryResult:
        """Discover integration tests in the project.

        Args:
            project_path: Path to project root

        Returns:
            TestDiscoveryResult with discovered tests
        """
        logger.info(f"Discovering integration tests in {project_path}")

        try:
            result = subprocess.run(
                [
                    "pytest",
                    str(project_path / "tests" / "integration"),
                    "--collect-only",
                    "-q",
                ],
                capture_output=True,
                text=True,
                timeout=60,
                cwd=project_path,
            )

            test_ids = []
            test_files = set()

            for line in result.stdout.strip().split("\n"):
                if "::" in line and not line.startswith("="):
                    test_ids.append(line.strip())
                    file_path = line.split("::")[0]
                    test_files.add(file_path)

            return TestDiscoveryResult(
                suite_type=self.suite_type,
                total_tests=len(test_ids),
                test_files=list(test_files),
                test_ids=test_ids,
                estimated_duration_seconds=len(test_ids) * 2.0,  # Estimate 2s per test
            )

        except subprocess.TimeoutExpired:
            logger.warning("Test discovery timed out")
            return TestDiscoveryResult(
                suite_type=self.suite_type,
                total_tests=0,
            )
        except FileNotFoundError:
            logger.warning("pytest not found")
            return TestDiscoveryResult(
                suite_type=self.suite_type,
                total_tests=0,
            )

    def run(
        self,
        project_path: Path,
        run_id: UUID,
        config: Optional[TestRunConfig] = None,
    ) -> TestSuiteExecution:
        """Run integration tests.

        Args:
            project_path: Path to project root
            run_id: Validation run ID
            config: Optional test configuration

        Returns:
            TestSuiteExecution with results

        Raises:
            TestExecutionError: If test execution fails
        """
        logger.info(f"Running integration tests for {project_path}")

        config = config or TestRunConfig(
            project_path=str(project_path),
            test_type=self.suite_type,
            timeout_seconds=600,  # Integration tests get longer timeout
        )

        execution = TestSuiteExecution(
            run_id=run_id,
            suite_type=self.suite_type,
            status=TestRunnerStatus.RUNNING,
        )

        try:
            # Build pytest command for integration tests
            cmd = [
                "pytest",
                str(project_path / "tests" / "integration"),
                "--tb=short",
                "-v",
                "-m", "integration or not unit",  # Run integration-marked tests
            ]

            if config.parallel:
                # Integration tests may have dependencies, use fewer workers
                cmd.extend(["-n", "2"])

            if config.coverage:
                cmd.extend([
                    "--cov=" + str(project_path / "src"),
                    "--cov-report=json",
                    "--cov-append",  # Append to existing coverage data
                ])

            # Run tests
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=config.timeout_seconds,
                cwd=project_path,
            )

            # Parse results
            execution = self._parse_results(execution, result)
            execution.status = TestRunnerStatus.COMPLETED

            return execution

        except subprocess.TimeoutExpired:
            execution.status = TestRunnerStatus.TIMEOUT
            raise TestExecutionError("integration", "Integration tests timed out")
        except FileNotFoundError:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("integration", "pytest not installed")
        except subprocess.SubprocessError as e:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("integration", str(e))

    def _parse_results(
        self,
        execution: TestSuiteExecution,
        result: subprocess.CompletedProcess,
    ) -> TestSuiteExecution:
        """Parse pytest output and update execution results."""
        # Parse stdout for counts
        import re

        for line in result.stdout.split("\n"):
            if "passed" in line or "failed" in line:
                passed_match = re.search(r"(\d+) passed", line)
                failed_match = re.search(r"(\d+) failed", line)
                skipped_match = re.search(r"(\d+) skipped", line)
                error_match = re.search(r"(\d+) error", line)

                if passed_match:
                    execution.passed = int(passed_match.group(1))
                if failed_match:
                    execution.failed = int(failed_match.group(1))
                if skipped_match:
                    execution.skipped = int(skipped_match.group(1))
                if error_match:
                    execution.errors = int(error_match.group(1))

                execution.total_tests = (
                    execution.passed
                    + execution.failed
                    + execution.skipped
                    + execution.errors
                )

        return execution


def get_integration_test_runner() -> IntegrationTestRunner:
    """Get integration test runner instance.

    Returns:
        IntegrationTestRunner instance
    """
    return IntegrationTestRunner()
