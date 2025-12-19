"""Unit test runner adapter.

This module provides the unit test runner using pytest as the test framework.
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


class UnitTestRunner:
    """Runner for unit tests using pytest."""

    def __init__(self):
        """Initialize the unit test runner."""
        self.settings = get_settings()
        self.suite_type = TestSuiteType.UNIT

    def discover(self, project_path: Path) -> TestDiscoveryResult:
        """Discover unit tests in the project.

        Args:
            project_path: Path to project root

        Returns:
            TestDiscoveryResult with discovered tests
        """
        logger.info(f"Discovering unit tests in {project_path}")

        try:
            result = subprocess.run(
                [
                    "pytest",
                    str(project_path / "tests" / "unit"),
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
                estimated_duration_seconds=len(test_ids) * 0.5,  # Estimate 0.5s per test
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
        """Run unit tests.

        Args:
            project_path: Path to project root
            run_id: Validation run ID
            config: Optional test configuration

        Returns:
            TestSuiteExecution with results

        Raises:
            TestExecutionError: If test execution fails
        """
        logger.info(f"Running unit tests for {project_path}")

        config = config or TestRunConfig(
            project_path=str(project_path),
            test_type=self.suite_type,
        )

        execution = TestSuiteExecution(
            run_id=run_id,
            suite_type=self.suite_type,
            status=TestRunnerStatus.RUNNING,
        )

        try:
            # Build pytest command
            cmd = [
                "pytest",
                str(project_path / "tests" / "unit"),
                "--tb=short",
                "-v",
                "--json-report",
                "--json-report-file=-",
            ]

            if config.parallel:
                cmd.extend(["-n", "auto"])

            if config.coverage:
                cmd.extend([
                    "--cov=" + str(project_path / "src"),
                    "--cov-report=json",
                ])

            if config.markers:
                cmd.extend(["-m", " or ".join(config.markers)])

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
            raise TestExecutionError("unit", "Unit tests timed out")
        except FileNotFoundError:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("unit", "pytest not installed")
        except subprocess.SubprocessError as e:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("unit", str(e))

    def _parse_results(
        self,
        execution: TestSuiteExecution,
        result: subprocess.CompletedProcess,
    ) -> TestSuiteExecution:
        """Parse pytest output and update execution results.

        Args:
            execution: Execution to update
            result: Subprocess result

        Returns:
            Updated TestSuiteExecution
        """
        # Try to parse JSON report from output
        try:
            # Find JSON in output (pytest-json-report outputs to stdout when file is -)
            json_start = result.stdout.find("{")
            if json_start >= 0:
                json_str = result.stdout[json_start:]
                report = json.loads(json_str)

                summary = report.get("summary", {})
                execution.total_tests = summary.get("total", 0)
                execution.passed = summary.get("passed", 0)
                execution.failed = summary.get("failed", 0)
                execution.skipped = summary.get("skipped", 0)
                execution.errors = summary.get("error", 0)

                # Parse individual test results
                for test in report.get("tests", []):
                    test_result = TestExecutionResult(
                        test_id=test.get("nodeid", "unknown"),
                        test_name=test.get("nodeid", "").split("::")[-1],
                        file_path=test.get("nodeid", "").split("::")[0],
                        status=self._map_outcome(test.get("outcome", "unknown")),
                        duration_seconds=test.get("duration", 0.0),
                        error_message=test.get("call", {}).get("longrepr"),
                    )
                    execution.results.append(test_result)

        except json.JSONDecodeError:
            # Fallback: parse stdout for basic counts
            self._parse_stdout_fallback(execution, result.stdout)

        return execution

    def _parse_stdout_fallback(
        self,
        execution: TestSuiteExecution,
        stdout: str,
    ) -> None:
        """Fallback parsing from stdout when JSON not available."""
        lines = stdout.split("\n")
        for line in lines:
            if "passed" in line or "failed" in line:
                # Try to extract counts from summary line
                # e.g., "===== 5 passed, 2 failed in 1.23s ====="
                import re

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

    def _map_outcome(self, outcome: str) -> TestStatus:
        """Map pytest outcome to TestStatus.

        Args:
            outcome: pytest outcome string

        Returns:
            TestStatus
        """
        mapping = {
            "passed": TestStatus.PASSED,
            "failed": TestStatus.FAILED,
            "error": TestStatus.ERROR,
            "skipped": TestStatus.SKIPPED,
        }
        return mapping.get(outcome, TestStatus.ERROR)


def get_unit_test_runner() -> UnitTestRunner:
    """Get unit test runner instance.

    Returns:
        UnitTestRunner instance
    """
    return UnitTestRunner()
