"""Tests for core data models."""

import pytest
from datetime import datetime, timedelta
from uuid import uuid4

from src.core.models import (
    TriggerSource,
    ValidationStatus,
    ValidationRun,
    TestSuiteResult,
    TestSuiteType,
    TestStatus,
    TestCaseResult,
    FlakyTestRecord,
    FlakyStatus,
)


class TestValidationRun:
    """Tests for ValidationRun model."""

    def test_create_validation_run(self):
        """Test creating a validation run."""
        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="my-repo",
            branch="main",
            commit_sha="abc123",
        )

        assert run.trigger_source == TriggerSource.MANUAL
        assert run.repository == "my-repo"
        assert run.branch == "main"
        assert run.commit_sha == "abc123"
        assert run.status == ValidationStatus.PENDING

    def test_start_validation_run(self):
        """Test starting a validation run."""
        run = ValidationRun(
            trigger_source=TriggerSource.CI_CD,
            repository="my-repo",
            branch="main",
            commit_sha="abc123",
        )

        run.start()

        assert run.status == ValidationStatus.RUNNING
        assert run.started_at is not None

    def test_complete_validation_run_success(self):
        """Test completing a validation run successfully."""
        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="my-repo",
            branch="main",
            commit_sha="abc123",
        )

        run.start()
        run.succeed()

        assert run.status == ValidationStatus.PASSED
        assert run.completed_at is not None
        assert run.duration_seconds is not None

    def test_complete_validation_run_failure(self):
        """Test failing a validation run."""
        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="my-repo",
            branch="main",
            commit_sha="abc123",
        )

        run.start()
        run.fail()

        assert run.status == ValidationStatus.FAILED


class TestTestSuiteResult:
    """Tests for TestSuiteResult model."""

    def test_create_test_suite_result(self):
        """Test creating a test suite result."""
        run_id = uuid4()
        result = TestSuiteResult(
            run_id=run_id,
            suite_type=TestSuiteType.UNIT,
            total_tests=10,
            passed_count=8,
            failed_count=2,
            skipped_count=0,
            error_count=0,
        )

        assert result.run_id == run_id
        assert result.suite_type == TestSuiteType.UNIT
        assert result.total_tests == 10
        assert result.passed_count == 8
        assert result.failed_count == 2

    def test_test_counts_validation(self):
        """Test that test counts must add up to total."""
        run_id = uuid4()

        with pytest.raises(ValueError, match="Test counts"):
            TestSuiteResult(
                run_id=run_id,
                suite_type=TestSuiteType.UNIT,
                total_tests=10,
                passed_count=5,
                failed_count=2,  # Doesn't add up to 10
                skipped_count=0,
                error_count=0,
            )

    def test_test_suite_with_test_cases(self):
        """Test adding test cases to suite result."""
        run_id = uuid4()

        test_cases = [
            TestCaseResult(
                test_id="test_module::test_function",
                test_name="test_function",
                status=TestStatus.PASSED,
                duration_seconds=0.5,
            ),
            TestCaseResult(
                test_id="test_module::test_failure",
                test_name="test_failure",
                status=TestStatus.FAILED,
                duration_seconds=1.2,
                error_message="AssertionError: Expected 1, got 2",
            ),
        ]

        result = TestSuiteResult(
            run_id=run_id,
            suite_type=TestSuiteType.UNIT,
            total_tests=2,
            passed_count=1,
            failed_count=1,
            skipped_count=0,
            error_count=0,
            test_cases=test_cases,
        )

        assert len(result.test_cases) == 2
        assert result.test_cases[0].status == TestStatus.PASSED
        assert result.test_cases[1].error_message is not None


class TestFlakyTestRecord:
    """Tests for FlakyTestRecord model."""

    def test_create_flaky_test_record(self):
        """Test creating a flaky test record."""
        record = FlakyTestRecord(
            test_id="test_module::test_flaky",
            confidence=0.95,
            pass_rate=0.7,
            sample_size=10,
        )

        assert record.test_id == "test_module::test_flaky"
        assert record.confidence == 0.95
        assert record.pass_rate == 0.7
        assert record.sample_size == 10
        assert record.status == FlakyStatus.ACTIVE_FLAKY

    def test_flaky_test_confidence_validation(self):
        """Test that confidence must be 0-1."""
        with pytest.raises(ValueError):
            FlakyTestRecord(
                test_id="test",
                confidence=1.5,
                pass_rate=0.7,
                sample_size=10,
            )

    def test_flaky_test_sample_size_validation(self):
        """Test that sample size must be >= 10."""
        with pytest.raises(ValueError):
            FlakyTestRecord(
                test_id="test",
                confidence=0.95,
                pass_rate=0.7,
                sample_size=5,
            )

    def test_flaky_test_with_execution_pattern(self):
        """Test flaky test with execution pattern."""
        record = FlakyTestRecord(
            test_id="test_module::test_flaky",
            confidence=0.95,
            pass_rate=0.7,
            sample_size=10,
            execution_pattern=[True, True, False, True, True, False, True, True, True, False],
        )

        assert len(record.execution_pattern) == 10
        # 7 passes out of 10 = 0.7 pass rate
        actual_pass_rate = sum(record.execution_pattern) / len(record.execution_pattern)
        assert actual_pass_rate == 0.7
