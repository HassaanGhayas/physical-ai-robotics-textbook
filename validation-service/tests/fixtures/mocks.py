"""Mock utilities for tests."""

from unittest.mock import MagicMock, AsyncMock
from typing import Any, Optional
from uuid import uuid4
from datetime import datetime

from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
    QualityReport,
    TestResult,
    SuiteType,
    FlakyTestInfo,
    FlakyStatus,
)


class MockStorage:
    """Mock storage class for testing."""

    def __init__(self):
        self._runs: dict[str, ValidationRun] = {}
        self._flaky_tests: list[FlakyTestInfo] = []
        self._data: dict[str, Any] = {}

    def get_run(self, run_id) -> Optional[ValidationRun]:
        """Get a run by ID."""
        return self._runs.get(str(run_id))

    def save_run(self, run: ValidationRun) -> None:
        """Save a run."""
        self._runs[str(run.run_id)] = run

    def list_runs(
        self,
        repository: Optional[str] = None,
        branch: Optional[str] = None,
        status: Optional[str] = None,
        limit: int = 20,
    ) -> list[ValidationRun]:
        """List runs with optional filters."""
        runs = list(self._runs.values())

        if repository:
            runs = [r for r in runs if r.repository == repository]
        if branch:
            runs = [r for r in runs if r.branch == branch]
        if status:
            runs = [r for r in runs if r.status.value == status]

        return runs[:limit]

    def get_latest_run(self) -> Optional[ValidationRun]:
        """Get the most recent run."""
        if not self._runs:
            return None
        return list(self._runs.values())[-1]

    def get_active_flaky_tests(self) -> list[FlakyTestInfo]:
        """Get active flaky tests."""
        return self._flaky_tests

    def save(self, key: str, data: Any) -> None:
        """Save arbitrary data."""
        self._data[key] = data

    def load(self, key: str) -> Optional[Any]:
        """Load arbitrary data."""
        return self._data.get(key)

    # Helper methods for tests
    def add_run(self, run: ValidationRun) -> None:
        """Add a run to the mock storage."""
        self._runs[str(run.run_id)] = run

    def add_flaky_test(self, test: FlakyTestInfo) -> None:
        """Add a flaky test to the mock storage."""
        self._flaky_tests.append(test)

    def clear(self) -> None:
        """Clear all stored data."""
        self._runs.clear()
        self._flaky_tests.clear()
        self._data.clear()


class MockPipeline:
    """Mock pipeline class for testing."""

    def __init__(self, default_status: ValidationStatus = ValidationStatus.PASSED):
        self.default_status = default_status
        self._calls: list[dict] = []

    async def run(
        self,
        project_path: str,
        repository: str,
        branch: str,
        commit_sha: str,
        trigger_source: TriggerSource,
        config: Any = None,
    ) -> ValidationRun:
        """Run the validation pipeline."""
        self._calls.append({
            "method": "run",
            "project_path": project_path,
            "repository": repository,
            "branch": branch,
            "commit_sha": commit_sha,
            "trigger_source": trigger_source,
            "config": config,
        })

        return ValidationRun(
            run_id=uuid4(),
            status=self.default_status,
            trigger_source=trigger_source,
            repository=repository,
            branch=branch,
            commit_sha=commit_sha,
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            duration_seconds=30.0,
        )

    async def run_quality_only(self, **kwargs) -> ValidationRun:
        """Run quality-only validation."""
        return await self.run(**kwargs, trigger_source=TriggerSource.MANUAL)

    async def run_tests_only(self, **kwargs) -> ValidationRun:
        """Run tests-only validation."""
        return await self.run(**kwargs, trigger_source=TriggerSource.MANUAL)

    async def run_deployment_only(self, **kwargs) -> ValidationRun:
        """Run deployment-only validation."""
        return await self.run(**kwargs, trigger_source=TriggerSource.MANUAL)

    @property
    def call_count(self) -> int:
        """Get the number of calls made."""
        return len(self._calls)

    @property
    def last_call(self) -> Optional[dict]:
        """Get the last call made."""
        return self._calls[-1] if self._calls else None

    def reset(self) -> None:
        """Reset call history."""
        self._calls.clear()


class MockSettings:
    """Mock settings class for testing."""

    def __init__(self):
        self.environment = "test"
        self.log_level = "DEBUG"
        self.enable_quality_validation = True
        self.enable_test_execution = True
        self.enable_deployment_validation = True
        self.pipeline_timeout_minutes = 15

        self.quality = MagicMock()
        self.quality.min_coverage_percentage = 80.0
        self.quality.max_cyclomatic_complexity = 10
        self.quality.max_line_length = 120
        self.quality.max_issues = 50

        self.testing = MagicMock()
        self.testing.parallel_execution = True
        self.testing.flaky_detection_enabled = True
        self.testing.retry_count = 3
        self.testing.timeout_per_test = 60

        self.deployment = MagicMock()
        self.deployment.require_passing_tests = True
        self.deployment.require_passing_quality = True
        self.deployment.allowed_environments = ["staging", "production"]


def create_mock_validator(passed: bool = True, issues: list = None):
    """Create a mock validator.

    Args:
        passed: Whether validation passes
        issues: List of issues to return

    Returns:
        Mock validator instance
    """
    validator = MagicMock()
    result = MagicMock()
    result.passed = passed
    result.issues = issues or []
    validator.validate = AsyncMock(return_value=result)
    return validator


def create_mock_runner(
    passed: bool = True,
    total_tests: int = 100,
    passed_tests: int = 100,
    failed_tests: int = 0,
):
    """Create a mock test runner.

    Args:
        passed: Whether tests pass
        total_tests: Total number of tests
        passed_tests: Number of passed tests
        failed_tests: Number of failed tests

    Returns:
        Mock runner instance
    """
    runner = MagicMock()
    result = MagicMock()
    result.passed = passed
    result.total_tests = total_tests
    result.passed_tests = passed_tests
    result.failed_tests = failed_tests
    result.skipped_tests = total_tests - passed_tests - failed_tests
    result.duration_seconds = 30.0
    result.failures = []
    runner.run = AsyncMock(return_value=result)
    return runner
