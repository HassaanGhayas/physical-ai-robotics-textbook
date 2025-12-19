"""Retry logic for flaky tests.

This module provides retry mechanisms for handling flaky tests,
including configurable retry strategies and backoff.
"""

import logging
import time
from dataclasses import dataclass
from datetime import datetime
from enum import Enum
from typing import Callable, Optional, TypeVar
from uuid import UUID

from src.core.config import get_settings
from src.core.models import TestStatus
from src.testing.models import TestExecutionResult


logger = logging.getLogger(__name__)

T = TypeVar("T")


class RetryStrategy(str, Enum):
    """Retry strategies for flaky tests."""

    FIXED = "fixed"  # Fixed delay between retries
    EXPONENTIAL = "exponential"  # Exponential backoff
    LINEAR = "linear"  # Linear increase in delay
    IMMEDIATE = "immediate"  # No delay between retries


@dataclass
class RetryConfig:
    """Configuration for test retries."""

    max_retries: int = 3
    strategy: RetryStrategy = RetryStrategy.FIXED
    base_delay_seconds: float = 1.0
    max_delay_seconds: float = 30.0
    retry_on_statuses: list[TestStatus] = None

    def __post_init__(self):
        if self.retry_on_statuses is None:
            self.retry_on_statuses = [TestStatus.FAILED, TestStatus.ERROR]


@dataclass
class RetryResult:
    """Result of a retry operation."""

    success: bool
    attempts: int
    final_result: TestExecutionResult
    attempt_history: list[TestExecutionResult]
    total_duration_seconds: float


class RetryHandler:
    """Service for handling test retries."""

    def __init__(self, config: Optional[RetryConfig] = None):
        """Initialize the retry handler.

        Args:
            config: Retry configuration
        """
        self.settings = get_settings()
        self.config = config or RetryConfig()

    def execute_with_retry(
        self,
        test_func: Callable[[], TestExecutionResult],
        test_id: str,
    ) -> RetryResult:
        """Execute a test with retry logic.

        Args:
            test_func: Function that executes the test
            test_id: Test identifier for logging

        Returns:
            RetryResult with final outcome
        """
        attempt_history = []
        start_time = datetime.utcnow()
        attempt = 0

        while attempt <= self.config.max_retries:
            attempt += 1
            logger.debug(f"Test {test_id}: attempt {attempt}/{self.config.max_retries + 1}")

            try:
                result = test_func()
                result.retry_count = attempt - 1  # Don't count first try as retry
                attempt_history.append(result)

                # Check if we should retry
                if result.status not in self.config.retry_on_statuses:
                    # Test passed or has non-retryable status
                    total_duration = (datetime.utcnow() - start_time).total_seconds()
                    return RetryResult(
                        success=result.status == TestStatus.PASSED,
                        attempts=attempt,
                        final_result=result,
                        attempt_history=attempt_history,
                        total_duration_seconds=total_duration,
                    )

                # Apply delay before retry
                if attempt <= self.config.max_retries:
                    delay = self._calculate_delay(attempt)
                    logger.debug(f"Test {test_id}: retrying in {delay:.2f}s")
                    time.sleep(delay)

            except Exception as e:
                logger.error(f"Test {test_id}: error on attempt {attempt}: {e}")
                error_result = TestExecutionResult(
                    test_id=test_id,
                    test_name=test_id,
                    status=TestStatus.ERROR,
                    duration_seconds=0.0,
                    error_message=str(e),
                    retry_count=attempt - 1,
                )
                attempt_history.append(error_result)

                if attempt <= self.config.max_retries:
                    delay = self._calculate_delay(attempt)
                    time.sleep(delay)

        # All retries exhausted
        total_duration = (datetime.utcnow() - start_time).total_seconds()
        final_result = attempt_history[-1] if attempt_history else TestExecutionResult(
            test_id=test_id,
            test_name=test_id,
            status=TestStatus.ERROR,
            duration_seconds=total_duration,
            error_message="All retry attempts exhausted",
            retry_count=self.config.max_retries,
        )

        return RetryResult(
            success=False,
            attempts=attempt,
            final_result=final_result,
            attempt_history=attempt_history,
            total_duration_seconds=total_duration,
        )

    def _calculate_delay(self, attempt: int) -> float:
        """Calculate delay before next retry.

        Args:
            attempt: Current attempt number

        Returns:
            Delay in seconds
        """
        if self.config.strategy == RetryStrategy.IMMEDIATE:
            return 0.0

        if self.config.strategy == RetryStrategy.FIXED:
            delay = self.config.base_delay_seconds

        elif self.config.strategy == RetryStrategy.LINEAR:
            delay = self.config.base_delay_seconds * attempt

        elif self.config.strategy == RetryStrategy.EXPONENTIAL:
            delay = self.config.base_delay_seconds * (2 ** (attempt - 1))

        else:
            delay = self.config.base_delay_seconds

        return min(delay, self.config.max_delay_seconds)

    def should_retry(self, result: TestExecutionResult, attempt: int) -> bool:
        """Determine if a test should be retried.

        Args:
            result: Test execution result
            attempt: Current attempt number

        Returns:
            True if should retry
        """
        if attempt >= self.config.max_retries + 1:
            return False

        return result.status in self.config.retry_on_statuses

    def mark_as_flaky(
        self,
        retry_result: RetryResult,
    ) -> bool:
        """Determine if a test should be marked as flaky based on retry results.

        A test is considered flaky if:
        - It eventually passed after failing
        - It has inconsistent results across attempts

        Args:
            retry_result: Result from retry execution

        Returns:
            True if test appears flaky
        """
        if len(retry_result.attempt_history) < 2:
            return False

        statuses = [r.status for r in retry_result.attempt_history]

        # Check for mixed pass/fail results
        passed_count = sum(1 for s in statuses if s == TestStatus.PASSED)
        failed_count = sum(1 for s in statuses if s in [TestStatus.FAILED, TestStatus.ERROR])

        # Flaky if we have both passes and failures
        return passed_count > 0 and failed_count > 0


class SelectiveRetryHandler(RetryHandler):
    """Retry handler that only retries known flaky tests."""

    def __init__(
        self,
        config: Optional[RetryConfig] = None,
        flaky_test_ids: Optional[set[str]] = None,
    ):
        """Initialize the selective retry handler.

        Args:
            config: Retry configuration
            flaky_test_ids: Set of known flaky test IDs
        """
        super().__init__(config)
        self.flaky_test_ids = flaky_test_ids or set()

    def execute_with_retry(
        self,
        test_func: Callable[[], TestExecutionResult],
        test_id: str,
    ) -> RetryResult:
        """Execute a test with retry only if it's known to be flaky.

        Args:
            test_func: Function that executes the test
            test_id: Test identifier

        Returns:
            RetryResult with final outcome
        """
        # Only retry known flaky tests
        if test_id not in self.flaky_test_ids:
            result = test_func()
            return RetryResult(
                success=result.status == TestStatus.PASSED,
                attempts=1,
                final_result=result,
                attempt_history=[result],
                total_duration_seconds=result.duration_seconds,
            )

        return super().execute_with_retry(test_func, test_id)

    def add_flaky_test(self, test_id: str) -> None:
        """Add a test ID to the flaky test set.

        Args:
            test_id: Test identifier to mark as flaky
        """
        self.flaky_test_ids.add(test_id)
        logger.info(f"Added {test_id} to flaky test set")

    def remove_flaky_test(self, test_id: str) -> None:
        """Remove a test ID from the flaky test set.

        Args:
            test_id: Test identifier to unmark as flaky
        """
        self.flaky_test_ids.discard(test_id)
        logger.info(f"Removed {test_id} from flaky test set")


def get_retry_handler(config: Optional[RetryConfig] = None) -> RetryHandler:
    """Get retry handler instance.

    Args:
        config: Optional retry configuration

    Returns:
        RetryHandler instance
    """
    return RetryHandler(config)


def get_selective_retry_handler(
    config: Optional[RetryConfig] = None,
    flaky_test_ids: Optional[set[str]] = None,
) -> SelectiveRetryHandler:
    """Get selective retry handler instance.

    Args:
        config: Optional retry configuration
        flaky_test_ids: Known flaky test IDs

    Returns:
        SelectiveRetryHandler instance
    """
    return SelectiveRetryHandler(config, flaky_test_ids)
