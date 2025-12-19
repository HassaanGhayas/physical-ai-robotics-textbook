"""Flaky test detection service.

This module provides statistical analysis to detect flaky tests based on
execution history and pass/fail patterns.
"""

import logging
import math
from datetime import datetime
from typing import Optional
from uuid import UUID, uuid4

from src.core.config import get_settings
from src.core.models import (
    TestCase,
    TestStatus,
    FlakyTestRecord,
    FlakyStatus,
    FailureMode,
    ExecutionRecord,
)


logger = logging.getLogger(__name__)


class FlakyTestDetector:
    """Service for detecting and tracking flaky tests."""

    # Minimum sample size for statistical significance
    MIN_SAMPLE_SIZE = 10

    # Confidence threshold for flaky detection (95%)
    CONFIDENCE_THRESHOLD = 0.95

    # False positive rate target (<5%)
    FALSE_POSITIVE_RATE = 0.05

    def __init__(self):
        """Initialize the flaky test detector."""
        self.settings = get_settings()

    def analyze_test(self, test_case: TestCase) -> Optional[FlakyTestRecord]:
        """Analyze a test case for flakiness.

        Args:
            test_case: Test case with execution history

        Returns:
            FlakyTestRecord if flaky, None otherwise
        """
        history = test_case.execution_history
        if len(history) < self.MIN_SAMPLE_SIZE:
            logger.debug(
                f"Insufficient history for {test_case.test_id}: "
                f"{len(history)} < {self.MIN_SAMPLE_SIZE}"
            )
            return None

        # Calculate pass rate
        passes = sum(1 for r in history if r.status == TestStatus.PASSED)
        total = len(history)
        pass_rate = passes / total

        # A test is potentially flaky if it's not 100% pass or 100% fail
        if pass_rate == 1.0 or pass_rate == 0.0:
            return None

        # Calculate confidence using Wilson score interval
        confidence = self._calculate_confidence(passes, total)

        if confidence < self.CONFIDENCE_THRESHOLD:
            logger.debug(
                f"Test {test_case.test_id} below confidence threshold: "
                f"{confidence:.2%} < {self.CONFIDENCE_THRESHOLD:.2%}"
            )
            return None

        # Build execution pattern
        execution_pattern = [r.status == TestStatus.PASSED for r in history]

        # Analyze failure modes
        failure_modes = self._analyze_failure_modes(test_case)

        # Calculate consecutive passes
        consecutive_passes = 0
        for record in reversed(history):
            if record.status == TestStatus.PASSED:
                consecutive_passes += 1
            else:
                break

        return FlakyTestRecord(
            test_id=test_case.test_id,
            detected_at=datetime.utcnow(),
            last_flake_at=self._find_last_failure(history),
            confidence=confidence,
            pass_rate=pass_rate,
            sample_size=total,
            consecutive_passes=consecutive_passes,
            status=FlakyStatus.ACTIVE_FLAKY,
            execution_pattern=execution_pattern,
            failure_modes=failure_modes,
        )

    def _calculate_confidence(self, successes: int, total: int) -> float:
        """Calculate confidence in flakiness using Wilson score interval.

        Args:
            successes: Number of passing runs
            total: Total number of runs

        Returns:
            Confidence level (0-1)
        """
        if total == 0:
            return 0.0

        p = successes / total
        z = 1.96  # 95% confidence z-score

        # Wilson score interval
        denominator = 1 + z * z / total
        center = (p + z * z / (2 * total)) / denominator
        margin = z * math.sqrt((p * (1 - p) + z * z / (4 * total)) / total) / denominator

        lower_bound = center - margin
        upper_bound = center + margin

        # Confidence is how far the pass rate is from 0% and 100%
        # A truly flaky test should have pass rate between these bounds
        if lower_bound > 0 and upper_bound < 1:
            # Both bounds are between 0 and 1, indicating potential flakiness
            return min(lower_bound, 1 - upper_bound) * 2
        else:
            return 0.0

    def _find_last_failure(self, history: list[ExecutionRecord]) -> datetime:
        """Find the timestamp of the last failure.

        Args:
            history: Execution history

        Returns:
            Timestamp of last failure
        """
        for record in reversed(history):
            if record.status != TestStatus.PASSED:
                return record.timestamp
        return datetime.utcnow()

    def _analyze_failure_modes(self, test_case: TestCase) -> list[FailureMode]:
        """Analyze distinct failure modes for a test.

        Args:
            test_case: Test case to analyze

        Returns:
            List of failure modes
        """
        # Group failures by error signature
        failures = {}
        total_failures = 0

        for record in test_case.execution_history:
            if record.status != TestStatus.PASSED:
                total_failures += 1
                # Use a simple hash of status as signature
                signature = str(hash(str(record.status)))[:8]

                if signature not in failures:
                    failures[signature] = {
                        "count": 0,
                        "sample": f"Test failed with status: {record.status.value}",
                    }
                failures[signature]["count"] += 1

        if total_failures == 0:
            return []

        return [
            FailureMode(
                error_signature=sig,
                frequency=data["count"] / total_failures,
                sample_error=data["sample"],
            )
            for sig, data in failures.items()
        ]

    def update_test_flakiness(
        self,
        test_case: TestCase,
        existing_record: Optional[FlakyTestRecord] = None,
    ) -> Optional[FlakyTestRecord]:
        """Update flakiness status for a test.

        Args:
            test_case: Test case with new execution data
            existing_record: Existing flaky record if any

        Returns:
            Updated FlakyTestRecord or None if not flaky
        """
        if existing_record is None:
            return self.analyze_test(test_case)

        # Update existing record with new data
        history = test_case.execution_history
        passes = sum(1 for r in history if r.status == TestStatus.PASSED)
        total = len(history)

        if total == 0:
            return existing_record

        pass_rate = passes / total
        confidence = self._calculate_confidence(passes, total)

        # Calculate consecutive passes
        consecutive_passes = 0
        for record in reversed(history):
            if record.status == TestStatus.PASSED:
                consecutive_passes += 1
            else:
                break

        # Update the record
        existing_record.pass_rate = pass_rate
        existing_record.confidence = confidence
        existing_record.sample_size = total
        existing_record.consecutive_passes = consecutive_passes
        existing_record.execution_pattern = [
            r.status == TestStatus.PASSED for r in history
        ]

        # Check if resolved (10 consecutive passes)
        if consecutive_passes >= 10:
            existing_record.status = FlakyStatus.RESOLVED

        # Update last flake time if there was a recent failure
        for record in reversed(history):
            if record.status != TestStatus.PASSED:
                existing_record.last_flake_at = record.timestamp
                break

        return existing_record

    def is_flaky(self, test_case: TestCase) -> bool:
        """Check if a test is flaky.

        Args:
            test_case: Test case to check

        Returns:
            True if test is flaky
        """
        record = self.analyze_test(test_case)
        return record is not None


def get_flaky_detector() -> FlakyTestDetector:
    """Get flaky test detector instance.

    Returns:
        FlakyTestDetector instance
    """
    return FlakyTestDetector()
