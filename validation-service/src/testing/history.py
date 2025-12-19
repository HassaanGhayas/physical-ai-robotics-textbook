"""Test execution history tracking.

This module provides tracking and querying of test execution history
for trend analysis and flaky test detection.
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    TestCase,
    TestStatus,
    TestSuiteType,
    ExecutionRecord,
)
from src.core.storage import Storage, get_storage


logger = logging.getLogger(__name__)


class TestHistoryTracker:
    """Service for tracking test execution history."""

    MAX_HISTORY_SIZE = 10  # Keep last 10 executions per test

    def __init__(self, storage: Optional[Storage] = None):
        """Initialize the history tracker.

        Args:
            storage: Storage backend (optional)
        """
        self.settings = get_settings()
        self.storage = storage or get_storage()
        self._test_cache: dict[str, TestCase] = {}

    def record_execution(
        self,
        test_id: str,
        test_name: str,
        test_type: TestSuiteType,
        file_path: str,
        run_id: UUID,
        status: TestStatus,
        duration_seconds: float,
        commit_sha: str,
    ) -> TestCase:
        """Record a test execution.

        Args:
            test_id: Unique test identifier
            test_name: Human-readable test name
            test_type: Test suite type
            file_path: Test file location
            run_id: Validation run ID
            status: Test result status
            duration_seconds: Execution time
            commit_sha: Git commit SHA

        Returns:
            Updated TestCase
        """
        # Get or create test case
        test_case = self._get_or_create_test(
            test_id=test_id,
            test_name=test_name,
            test_type=test_type,
            file_path=file_path,
        )

        # Create execution record
        record = ExecutionRecord(
            run_id=run_id,
            timestamp=datetime.utcnow(),
            status=status,
            duration_seconds=duration_seconds,
            commit_sha=commit_sha,
        )

        # Add to history (limited to MAX_HISTORY_SIZE)
        test_case.execution_history.append(record)
        if len(test_case.execution_history) > self.MAX_HISTORY_SIZE:
            test_case.execution_history = test_case.execution_history[-self.MAX_HISTORY_SIZE:]

        # Update computed fields
        test_case.last_execution = record.timestamp
        test_case.average_duration_seconds = self._calculate_average_duration(test_case)
        test_case.flakiness_score = self._calculate_flakiness_score(test_case)
        test_case.is_flaky = test_case.flakiness_score > 0.1  # 10% threshold

        # Save to storage
        self._save_test(test_case)

        return test_case

    def _get_or_create_test(
        self,
        test_id: str,
        test_name: str,
        test_type: TestSuiteType,
        file_path: str,
    ) -> TestCase:
        """Get existing test case or create new one.

        Args:
            test_id: Unique test identifier
            test_name: Human-readable test name
            test_type: Test suite type
            file_path: Test file location

        Returns:
            TestCase
        """
        # Check cache first
        if test_id in self._test_cache:
            return self._test_cache[test_id]

        # Try to load from storage
        test_case = self._load_test(test_id)
        if test_case:
            self._test_cache[test_id] = test_case
            return test_case

        # Create new test case
        test_case = TestCase(
            test_id=test_id,
            test_name=test_name,
            test_type=test_type,
            file_path=file_path,
            target_functionality=self._infer_functionality(file_path),
        )
        self._test_cache[test_id] = test_case
        return test_case

    def _infer_functionality(self, file_path: str) -> str:
        """Infer target functionality from file path.

        Args:
            file_path: Test file path

        Returns:
            Inferred functionality description
        """
        # Extract module name from path
        parts = Path(file_path).parts
        for part in reversed(parts):
            if part.startswith("test_"):
                return part.replace("test_", "").replace(".py", "")
        return "unknown"

    def _calculate_average_duration(self, test_case: TestCase) -> float:
        """Calculate average execution duration.

        Args:
            test_case: Test case with history

        Returns:
            Average duration in seconds
        """
        if not test_case.execution_history:
            return 0.0

        total = sum(r.duration_seconds for r in test_case.execution_history)
        return total / len(test_case.execution_history)

    def _calculate_flakiness_score(self, test_case: TestCase) -> float:
        """Calculate flakiness score based on execution history.

        Args:
            test_case: Test case with history

        Returns:
            Flakiness score (0-1)
        """
        history = test_case.execution_history
        if len(history) < 2:
            return 0.0

        # Count status transitions (changes between pass/fail)
        transitions = 0
        for i in range(1, len(history)):
            if history[i].status != history[i - 1].status:
                transitions += 1

        # Flakiness score = transition rate
        max_transitions = len(history) - 1
        return transitions / max_transitions if max_transitions > 0 else 0.0

    def _load_test(self, test_id: str) -> Optional[TestCase]:
        """Load test case from storage.

        Args:
            test_id: Test identifier

        Returns:
            TestCase or None
        """
        try:
            data = self.storage.load(f"tests/{test_id}")
            if data:
                return TestCase.model_validate(data)
        except Exception as e:
            logger.debug(f"Failed to load test {test_id}: {e}")
        return None

    def _save_test(self, test_case: TestCase) -> None:
        """Save test case to storage.

        Args:
            test_case: Test case to save
        """
        try:
            self.storage.save(
                f"tests/{test_case.test_id}",
                test_case.model_dump(mode="json"),
            )
        except Exception as e:
            logger.error(f"Failed to save test {test_case.test_id}: {e}")

    def get_test_history(
        self,
        test_id: str,
        limit: int = 10,
    ) -> Optional[TestCase]:
        """Get test case with execution history.

        Args:
            test_id: Test identifier
            limit: Maximum history entries to return

        Returns:
            TestCase or None
        """
        test_case = self._load_test(test_id)
        if test_case and limit < len(test_case.execution_history):
            test_case.execution_history = test_case.execution_history[-limit:]
        return test_case

    def get_flaky_tests(
        self,
        threshold: float = 0.1,
    ) -> list[TestCase]:
        """Get all flaky tests above threshold.

        Args:
            threshold: Minimum flakiness score

        Returns:
            List of flaky test cases
        """
        flaky_tests = []

        # Scan storage for all tests
        test_ids = self.storage.list("tests/")
        for test_id in test_ids:
            test_case = self._load_test(test_id.replace("tests/", ""))
            if test_case and test_case.flakiness_score >= threshold:
                flaky_tests.append(test_case)

        return flaky_tests

    def get_recent_failures(
        self,
        limit: int = 10,
    ) -> list[tuple[str, ExecutionRecord]]:
        """Get recent test failures.

        Args:
            limit: Maximum failures to return

        Returns:
            List of (test_id, ExecutionRecord) tuples
        """
        failures = []

        test_ids = self.storage.list("tests/")
        for test_id in test_ids:
            test_case = self._load_test(test_id.replace("tests/", ""))
            if test_case:
                for record in test_case.execution_history:
                    if record.status != TestStatus.PASSED:
                        failures.append((test_case.test_id, record))

        # Sort by timestamp descending
        failures.sort(key=lambda x: x[1].timestamp, reverse=True)
        return failures[:limit]


def get_history_tracker() -> TestHistoryTracker:
    """Get test history tracker instance.

    Returns:
        TestHistoryTracker instance
    """
    return TestHistoryTracker()
