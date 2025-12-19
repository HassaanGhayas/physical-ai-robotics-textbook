"""Sample data fixtures for tests."""

from datetime import datetime
from pathlib import Path
from uuid import uuid4
from typing import Optional

from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
    QualityReport,
    QualityIssue,
    IssueSeverity,
    TestResult,
    SuiteType,
    TestFailure,
    FlakyTestInfo,
    FlakyStatus,
    DeploymentChecklist,
    DeploymentCheck,
)


def create_sample_validation_run(
    run_id: Optional[str] = None,
    status: ValidationStatus = ValidationStatus.PASSED,
    trigger_source: TriggerSource = TriggerSource.MANUAL,
    repository: str = "test-repo",
    branch: str = "main",
    commit_sha: str = "abc123def456789",
    include_quality: bool = True,
    include_tests: bool = True,
    include_deployment: bool = False,
) -> ValidationRun:
    """Create a sample validation run for testing.

    Args:
        run_id: Optional run ID (generates random if not provided)
        status: Validation status
        trigger_source: How the validation was triggered
        repository: Repository name
        branch: Branch name
        commit_sha: Commit SHA
        include_quality: Include quality report
        include_tests: Include test results
        include_deployment: Include deployment checklist

    Returns:
        A ValidationRun instance with sample data
    """
    run = ValidationRun(
        run_id=uuid4() if run_id is None else run_id,
        status=status,
        trigger_source=trigger_source,
        repository=repository,
        branch=branch,
        commit_sha=commit_sha,
        started_at=datetime.utcnow(),
        completed_at=datetime.utcnow(),
        duration_seconds=45.5,
    )

    if include_quality:
        run.quality_report = create_sample_quality_report()

    if include_tests:
        run.test_results = [
            create_sample_test_result(SuiteType.UNIT),
            create_sample_test_result(SuiteType.INTEGRATION),
        ]

    if include_deployment:
        run.deployment_checklist = create_sample_deployment_checklist()

    return run


def create_sample_quality_report(
    coverage: float = 85.0,
    passed: bool = True,
    num_issues: int = 1,
) -> QualityReport:
    """Create a sample quality report for testing.

    Args:
        coverage: Coverage percentage
        passed: Whether quality checks passed
        num_issues: Number of issues to include

    Returns:
        A QualityReport instance with sample data
    """
    issues = []
    if num_issues > 0:
        severities = [IssueSeverity.WARNING, IssueSeverity.ERROR, IssueSeverity.INFO]
        for i in range(num_issues):
            issues.append(
                QualityIssue(
                    severity=severities[i % len(severities)],
                    message=f"Sample issue {i + 1}",
                    file_path=f"src/module_{i + 1}.py",
                    line_number=(i + 1) * 10,
                )
            )

    return QualityReport(
        coverage_percentage=coverage,
        passed=passed,
        issues=issues,
    )


def create_sample_test_result(
    suite_type: SuiteType = SuiteType.UNIT,
    total_tests: int = 100,
    passed_tests: int = 98,
    failed_tests: int = 2,
    skipped_tests: int = 0,
    include_failures: bool = True,
) -> TestResult:
    """Create a sample test result for testing.

    Args:
        suite_type: Type of test suite
        total_tests: Total number of tests
        passed_tests: Number of passed tests
        failed_tests: Number of failed tests
        skipped_tests: Number of skipped tests
        include_failures: Include failure details

    Returns:
        A TestResult instance with sample data
    """
    failures = []
    if include_failures and failed_tests > 0:
        for i in range(min(failed_tests, 3)):  # Max 3 sample failures
            failures.append(
                TestFailure(
                    test_name=f"test_sample_{i + 1}",
                    error_message=f"AssertionError: Sample failure {i + 1}",
                    file_path=f"tests/test_module_{i + 1}.py",
                    line_number=(i + 1) * 20,
                )
            )

    return TestResult(
        suite_type=suite_type,
        total_tests=total_tests,
        passed_tests=passed_tests,
        failed_tests=failed_tests,
        skipped_tests=skipped_tests,
        duration_seconds=30.0 if suite_type == SuiteType.UNIT else 60.0,
        passed=failed_tests == 0,
        failures=failures,
    )


def create_sample_flaky_test(
    test_id: str = "test_module::TestClass::test_method",
    pass_rate: float = 0.75,
    sample_size: int = 20,
    confidence: float = 0.92,
    status: FlakyStatus = FlakyStatus.ACTIVE_FLAKY,
) -> FlakyTestInfo:
    """Create a sample flaky test info for testing.

    Args:
        test_id: Test identifier
        pass_rate: Pass rate (0-1)
        sample_size: Number of runs sampled
        confidence: Confidence score (0-1)
        status: Flaky test status

    Returns:
        A FlakyTestInfo instance with sample data
    """
    return FlakyTestInfo(
        test_id=test_id,
        pass_rate=pass_rate,
        sample_size=sample_size,
        confidence=confidence,
        status=status,
    )


def create_sample_deployment_checklist(
    passed: bool = True,
    num_checks: int = 5,
) -> DeploymentChecklist:
    """Create a sample deployment checklist for testing.

    Args:
        passed: Whether all checks passed
        num_checks: Number of checks to include

    Returns:
        A DeploymentChecklist instance with sample data
    """
    check_names = [
        "Configuration validation",
        "Dependency compatibility",
        "Security scan",
        "Database migrations",
        "Environment variables",
    ]

    checks = []
    for i in range(min(num_checks, len(check_names))):
        check_passed = passed or i < num_checks - 1  # Last one fails if not passed
        checks.append(
            DeploymentCheck(
                name=check_names[i],
                passed=check_passed,
                message="Check passed" if check_passed else "Check failed",
            )
        )

    return DeploymentChecklist(
        passed=passed,
        checks=checks,
    )


def create_sample_project(base_path: Path) -> Path:
    """Create a sample project structure for testing.

    Args:
        base_path: Base path for the project (usually tmp_path)

    Returns:
        Path to the created project
    """
    # Create directory structure
    src_dir = base_path / "src"
    src_dir.mkdir(parents=True, exist_ok=True)

    tests_dir = base_path / "tests"
    tests_dir.mkdir(parents=True, exist_ok=True)

    git_dir = base_path / ".git"
    git_dir.mkdir(parents=True, exist_ok=True)

    # Create source files
    (src_dir / "__init__.py").write_text("")

    (src_dir / "main.py").write_text('''
"""Main application module."""

from typing import Optional


def greet(name: str = "World") -> str:
    """Return a greeting message.

    Args:
        name: Name to greet

    Returns:
        Greeting message
    """
    return f"Hello, {name}!"


def add(a: int, b: int) -> int:
    """Add two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Sum of a and b
    """
    return a + b


def divide(a: float, b: float) -> Optional[float]:
    """Divide two numbers.

    Args:
        a: Dividend
        b: Divisor

    Returns:
        Result of division, or None if divisor is zero
    """
    if b == 0:
        return None
    return a / b
''')

    # Create test files
    (tests_dir / "__init__.py").write_text("")

    (tests_dir / "test_main.py").write_text('''
"""Tests for main module."""

import pytest
from src.main import greet, add, divide


class TestGreet:
    """Tests for greet function."""

    def test_greet_default(self):
        assert greet() == "Hello, World!"

    def test_greet_custom_name(self):
        assert greet("Alice") == "Hello, Alice!"


class TestAdd:
    """Tests for add function."""

    def test_add_positive(self):
        assert add(2, 3) == 5

    def test_add_negative(self):
        assert add(-1, 1) == 0

    def test_add_zero(self):
        assert add(0, 0) == 0


class TestDivide:
    """Tests for divide function."""

    def test_divide_normal(self):
        assert divide(10, 2) == 5.0

    def test_divide_by_zero(self):
        assert divide(10, 0) is None

    def test_divide_float(self):
        assert divide(7.5, 2.5) == 3.0
''')

    # Create configuration files
    (base_path / "pyproject.toml").write_text('''
[project]
name = "test-project"
version = "1.0.0"
description = "A sample test project"
requires-python = ">=3.10"

[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = "test_*.py"

[tool.ruff]
line-length = 120
target-version = "py310"

[tool.mypy]
python_version = "3.10"
strict = true
''')

    (base_path / "requirements.txt").write_text('''
pytest>=7.0.0
pytest-asyncio>=0.21.0
''')

    return base_path
