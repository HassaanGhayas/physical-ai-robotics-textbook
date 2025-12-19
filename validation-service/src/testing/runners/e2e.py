"""End-to-end test runner adapter.

This module provides the E2E test runner supporting Playwright and pytest.
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


class E2ETestRunner:
    """Runner for end-to-end tests using pytest-playwright or similar."""

    def __init__(self):
        """Initialize the E2E test runner."""
        self.settings = get_settings()
        self.suite_type = TestSuiteType.E2E

    def discover(self, project_path: Path) -> TestDiscoveryResult:
        """Discover E2E tests in the project.

        Args:
            project_path: Path to project root

        Returns:
            TestDiscoveryResult with discovered tests
        """
        logger.info(f"Discovering E2E tests in {project_path}")

        e2e_path = project_path / "tests" / "e2e"
        if not e2e_path.exists():
            return TestDiscoveryResult(
                suite_type=self.suite_type,
                total_tests=0,
            )

        try:
            result = subprocess.run(
                [
                    "pytest",
                    str(e2e_path),
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
                estimated_duration_seconds=len(test_ids) * 10.0,  # Estimate 10s per E2E test
            )

        except subprocess.TimeoutExpired:
            logger.warning("E2E test discovery timed out")
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
        """Run E2E tests.

        Args:
            project_path: Path to project root
            run_id: Validation run ID
            config: Optional test configuration

        Returns:
            TestSuiteExecution with results

        Raises:
            TestExecutionError: If test execution fails
        """
        logger.info(f"Running E2E tests for {project_path}")

        config = config or TestRunConfig(
            project_path=str(project_path),
            test_type=self.suite_type,
            timeout_seconds=900,  # E2E tests get 15 min timeout
            parallel=False,  # E2E tests typically run sequentially
        )

        execution = TestSuiteExecution(
            run_id=run_id,
            suite_type=self.suite_type,
            status=TestRunnerStatus.RUNNING,
        )

        e2e_path = project_path / "tests" / "e2e"
        if not e2e_path.exists():
            execution.status = TestRunnerStatus.COMPLETED
            return execution

        try:
            # Build pytest command for E2E tests
            cmd = [
                "pytest",
                str(e2e_path),
                "--tb=short",
                "-v",
                "-m", "e2e or not (unit or integration)",
            ]

            # E2E tests often need headed browser for debugging
            if config.verbose:
                cmd.append("--headed")

            # E2E tests typically run sequentially to avoid resource conflicts
            if config.parallel:
                cmd.extend(["-n", "1"])  # Single worker for stability

            # Set Playwright browser if applicable
            cmd.extend(["--browser", "chromium"])

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
            raise TestExecutionError("e2e", "E2E tests timed out")
        except FileNotFoundError:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("e2e", "pytest not installed")
        except subprocess.SubprocessError as e:
            execution.status = TestRunnerStatus.FAILED
            raise TestExecutionError("e2e", str(e))

    def _parse_results(
        self,
        execution: TestSuiteExecution,
        result: subprocess.CompletedProcess,
    ) -> TestSuiteExecution:
        """Parse pytest output and update execution results."""
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

    def setup_browser(self, browser: str = "chromium") -> bool:
        """Install browser for E2E testing.

        Args:
            browser: Browser to install (chromium, firefox, webkit)

        Returns:
            True if successful
        """
        try:
            subprocess.run(
                ["playwright", "install", browser],
                capture_output=True,
                timeout=300,
                check=True,
            )
            return True
        except (subprocess.SubprocessError, FileNotFoundError):
            logger.warning(f"Failed to install {browser}")
            return False


def get_e2e_test_runner() -> E2ETestRunner:
    """Get E2E test runner instance.

    Returns:
        E2ETestRunner instance
    """
    return E2ETestRunner()
