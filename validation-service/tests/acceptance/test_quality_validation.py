"""Acceptance tests for User Story 1 - Automated Code Quality Validation.

These tests verify the acceptance scenarios defined in the specification:
1. Style violations are identified with line numbers
2. Insufficient coverage is reported with percentages
3. Code meeting standards confirms passing status
"""

import pytest
from pathlib import Path
from uuid import uuid4
from unittest.mock import patch, Mock
import json

from src.core.models import (
    CheckStatus,
    CodeQualityReport,
    ValidationRun,
    TriggerSource,
    Severity,
)
from src.quality.validator import QualityValidator
from src.quality.models import (
    LintingResults,
    ComplexityResults,
    CoverageResults,
    SecurityResults,
    Violation,
)


class TestAcceptanceScenario1_StyleViolations:
    """
    Acceptance Scenario 1:
    Given a code change with style violations,
    When the developer submits it for validation,
    Then the system identifies all style issues with line numbers and provides actionable feedback
    """

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_identifies_style_violations_with_line_numbers(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Given code with style violations, system identifies them with line numbers."""
        # Setup: Mock linter to return violations
        violations_json = json.dumps([
            {
                "filename": "src/main.py",
                "location": {"row": 10, "column": 5},
                "code": "E501",
                "message": "Line too long (120 > 88 characters)",
            },
            {
                "filename": "src/utils.py",
                "location": {"row": 25, "column": 1},
                "code": "F401",
                "message": "Module 'os' imported but unused",
            },
            {
                "filename": "src/main.py",
                "location": {"row": 50, "column": 10},
                "code": "E302",
                "message": "Expected 2 blank lines, found 1",
            },
        ])

        mock_linter.return_value = Mock(stdout=violations_json, returncode=1)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(stdout=json.dumps({"totals": {"percent_covered": 80.0}}), returncode=0),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        # Execute: Run validation
        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify: All violations identified with line numbers
        assert report.linting.total_violations == 3
        assert report.linting.error_count >= 1  # E501, F401, E302 are errors

        # Verify each violation has line number
        for violation in report.linting.violations:
            assert violation.line >= 1, "Each violation must have a line number"
            assert violation.file, "Each violation must identify the file"
            assert violation.message, "Each violation must have actionable message"
            assert violation.rule, "Each violation must identify the rule"

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_provides_actionable_feedback(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Violations include actionable feedback (rule codes, messages)."""
        violations_json = json.dumps([
            {
                "filename": "src/main.py",
                "location": {"row": 15, "column": 1},
                "code": "E501",
                "message": "Line too long (120 > 88 characters)",
            },
        ])

        mock_linter.return_value = Mock(stdout=violations_json, returncode=1)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(stdout=json.dumps({"totals": {"percent_covered": 80.0}}), returncode=0),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify actionable feedback
        violation = report.linting.violations[0]
        assert violation.rule == "E501", "Rule code should be provided"
        assert "too long" in violation.message.lower(), "Message should be actionable"


class TestAcceptanceScenario2_CoverageReporting:
    """
    Acceptance Scenario 2:
    Given a code change with insufficient test coverage,
    When the developer runs validation,
    Then the system reports current coverage percentage and identifies untested code paths
    """

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_reports_coverage_percentage(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Given insufficient coverage, system reports coverage percentage."""
        mock_linter.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),  # pytest run
            Mock(
                stdout=json.dumps({
                    "totals": {
                        "percent_covered": 45.5,
                        "percent_covered_branches": 35.0,
                        "missing_lines": 250,
                    }
                }),
                returncode=0,
            ),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify coverage percentage is reported
        assert report.coverage.line_coverage_percent == 45.5
        assert report.coverage.branch_coverage_percent == 35.0
        assert report.coverage.uncovered_lines == 250

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_identifies_untested_code(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """System identifies uncovered lines count."""
        mock_linter.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(
                stdout=json.dumps({
                    "totals": {
                        "percent_covered": 60.0,
                        "missing_lines": 100,
                    }
                }),
                returncode=0,
            ),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify uncovered lines are identified
        assert report.coverage.uncovered_lines == 100


class TestAcceptanceScenario3_PassingStatus:
    """
    Acceptance Scenario 3:
    Given a code change meeting all quality standards,
    When validation runs,
    Then the system confirms passing status and allows progression to code review
    """

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_confirms_passing_status(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Given code meeting standards, system confirms passing status."""
        # Setup: All checks pass
        mock_linter.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity.return_value = Mock(
            stdout=json.dumps({
                "src/main.py": [
                    {"name": "main", "complexity": 3, "rank": "A", "lineno": 1}
                ]
            }),
            returncode=0,
        )
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(
                stdout=json.dumps({
                    "totals": {
                        "percent_covered": 85.0,
                        "percent_covered_branches": 80.0,
                    }
                }),
                returncode=0,
            ),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify passing status
        assert report.status == CheckStatus.PASSED
        assert report.linting.total_violations == 0
        assert report.linting.error_count == 0
        assert report.security.critical_count == 0

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_quality_gate_passed(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Quality gate passes when all criteria met."""
        mock_linter.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(
                stdout=json.dumps({
                    "totals": {"percent_covered": 90.0}
                }),
                returncode=0,
            ),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify quality gate passes (no blocking issues)
        assert report.linting.error_count == 0
        assert report.security.critical_count == 0


class TestEdgeCases:
    """Test edge cases for quality validation."""

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_handles_empty_project(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """System handles empty project gracefully."""
        mock_linter.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage.side_effect = [
            Mock(stdout="", returncode=0),
            Mock(stdout="", returncode=0),
        ]
        mock_security.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = validator.validate(
                project_path=Path("/empty/project"),
                run_id=run_id,
            )

        # Should return valid report even for empty project
        assert report is not None
        assert report.run_id == run_id

    def test_handles_nonexistent_path(self):
        """System handles non-existent project path."""
        validator = QualityValidator()
        run_id = uuid4()

        report = validator.validate(
            project_path=Path("/nonexistent/path"),
            run_id=run_id,
        )

        assert report.status == CheckStatus.FAILED
