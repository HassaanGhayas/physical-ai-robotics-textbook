"""Tests for quality models."""

import pytest
from uuid import uuid4

from src.core.models import (
    CheckStatus,
    CodeQualityReport,
    ComplexityResults,
    CoverageResults,
    LintingResults,
    Severity,
    Violation,
    Vulnerability,
)


class TestViolation:
    """Tests for Violation model."""

    def test_create_violation(self):
        """Test creating a violation."""
        violation = Violation(
            file="src/main.py",
            line=10,
            column=5,
            rule="E501",
            severity=Severity.ERROR,
            message="Line too long",
        )

        assert violation.file == "src/main.py"
        assert violation.line == 10
        assert violation.column == 5
        assert violation.rule == "E501"
        assert violation.severity == Severity.ERROR
        assert violation.message == "Line too long"

    def test_violation_optional_column(self):
        """Test violation without column."""
        violation = Violation(
            file="src/main.py",
            line=10,
            rule="E501",
            severity=Severity.WARNING,
            message="Line too long",
        )

        assert violation.column is None

    def test_violation_line_validation(self):
        """Test that line must be >= 1."""
        with pytest.raises(ValueError):
            Violation(
                file="src/main.py",
                line=0,
                rule="E501",
                severity=Severity.ERROR,
                message="Line too long",
            )


class TestVulnerability:
    """Tests for Vulnerability model."""

    def test_create_vulnerability(self):
        """Test creating a vulnerability."""
        vuln = Vulnerability(
            package="requests",
            version="2.25.0",
            severity=Severity.HIGH,
            cve_id="CVE-2021-12345",
            description="Security issue in requests",
            fixed_version="2.26.0",
        )

        assert vuln.package == "requests"
        assert vuln.version == "2.25.0"
        assert vuln.severity == Severity.HIGH
        assert vuln.cve_id == "CVE-2021-12345"
        assert vuln.fixed_version == "2.26.0"

    def test_vulnerability_optional_fields(self):
        """Test vulnerability with optional fields omitted."""
        vuln = Vulnerability(
            package="requests",
            version="2.25.0",
            severity=Severity.MEDIUM,
            description="Security issue",
        )

        assert vuln.cve_id is None
        assert vuln.fixed_version is None


class TestLintingResults:
    """Tests for LintingResults model."""

    def test_empty_linting_results(self):
        """Test creating empty linting results."""
        results = LintingResults()

        assert results.violations == []
        assert results.total_violations == 0
        assert results.error_count == 0
        assert results.warning_count == 0

    def test_linting_results_with_violations(self):
        """Test linting results with violations."""
        violations = [
            Violation(
                file="src/main.py",
                line=10,
                rule="E501",
                severity=Severity.ERROR,
                message="Line too long",
            ),
            Violation(
                file="src/utils.py",
                line=20,
                rule="W503",
                severity=Severity.WARNING,
                message="Line break before operator",
            ),
        ]

        results = LintingResults(
            violations=violations,
            total_violations=2,
            error_count=1,
            warning_count=1,
        )

        assert len(results.violations) == 2
        assert results.total_violations == 2
        assert results.error_count == 1
        assert results.warning_count == 1


class TestComplexityResults:
    """Tests for ComplexityResults model."""

    def test_empty_complexity_results(self):
        """Test creating empty complexity results."""
        results = ComplexityResults()

        assert results.average_complexity == 0.0
        assert results.max_complexity == 0
        assert results.high_complexity_functions == []

    def test_complexity_results_with_data(self):
        """Test complexity results with data."""
        results = ComplexityResults(
            average_complexity=5.5,
            max_complexity=15,
            high_complexity_functions=["src/main.py::process_data"],
        )

        assert results.average_complexity == 5.5
        assert results.max_complexity == 15
        assert len(results.high_complexity_functions) == 1

    def test_complexity_non_negative_validation(self):
        """Test that complexity values must be non-negative."""
        with pytest.raises(ValueError):
            ComplexityResults(average_complexity=-1.0)


class TestCoverageResults:
    """Tests for CoverageResults model."""

    def test_empty_coverage_results(self):
        """Test creating empty coverage results."""
        results = CoverageResults()

        assert results.line_coverage_percent == 0.0
        assert results.branch_coverage_percent == 0.0
        assert results.uncovered_lines == 0

    def test_coverage_results_with_data(self):
        """Test coverage results with data."""
        results = CoverageResults(
            line_coverage_percent=85.5,
            branch_coverage_percent=75.0,
            uncovered_lines=150,
        )

        assert results.line_coverage_percent == 85.5
        assert results.branch_coverage_percent == 75.0
        assert results.uncovered_lines == 150

    def test_coverage_percentage_validation(self):
        """Test that coverage percentages are 0-100."""
        with pytest.raises(ValueError):
            CoverageResults(line_coverage_percent=150.0)


class TestCodeQualityReport:
    """Tests for CodeQualityReport model."""

    def test_create_quality_report(self):
        """Test creating a quality report."""
        run_id = uuid4()
        report = CodeQualityReport(run_id=run_id)

        assert report.run_id == run_id
        assert report.status == CheckStatus.PASSED
        assert report.linting.total_violations == 0
        assert report.complexity.average_complexity == 0.0
        assert report.coverage.line_coverage_percent == 0.0

    def test_quality_report_with_results(self):
        """Test quality report with actual results."""
        run_id = uuid4()

        violations = [
            Violation(
                file="src/main.py",
                line=10,
                rule="E501",
                severity=Severity.ERROR,
                message="Line too long",
            ),
        ]

        report = CodeQualityReport(
            run_id=run_id,
            status=CheckStatus.WARNING,
            linting=LintingResults(
                violations=violations,
                total_violations=1,
                error_count=1,
                warning_count=0,
            ),
            complexity=ComplexityResults(
                average_complexity=7.5,
                max_complexity=12,
            ),
            coverage=CoverageResults(
                line_coverage_percent=78.5,
            ),
        )

        assert report.status == CheckStatus.WARNING
        assert report.linting.total_violations == 1
        assert report.complexity.max_complexity == 12
        assert report.coverage.line_coverage_percent == 78.5
