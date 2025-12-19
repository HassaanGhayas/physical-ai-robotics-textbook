"""Tests for the coverage calculator service."""

import json
import pytest
from unittest.mock import Mock, patch
from pathlib import Path

from src.core.exceptions import CoverageError
from src.quality.coverage import CoverageCalculator, get_coverage_calculator
from src.quality.models import CoverageResults, CoverageByType


class TestCoverageCalculator:
    """Tests for CoverageCalculator."""

    def test_init(self):
        """Test coverage calculator initialization."""
        calculator = CoverageCalculator()
        assert calculator.settings is not None
        assert calculator.min_coverage >= 0

    @patch("src.quality.coverage.subprocess.run")
    def test_calculate_success(self, mock_run):
        """Test calculating coverage successfully."""
        coverage_output = json.dumps({
            "totals": {
                "percent_covered": 85.5,
                "percent_covered_branches": 75.0,
                "missing_lines": 150,
            },
        })

        # First call runs tests, second generates report
        mock_run.side_effect = [
            Mock(stdout="", stderr="", returncode=0),  # pytest run
            Mock(stdout=coverage_output, stderr="", returncode=0),  # coverage json
        ]

        calculator = CoverageCalculator()
        result = calculator.calculate(Path("/test/project"))

        assert result.line_coverage_percent == 85.5
        assert result.branch_coverage_percent == 75.0
        assert result.uncovered_lines == 150
        assert result.by_test_type.unit == 85.5

    @patch("src.quality.coverage.subprocess.run")
    def test_calculate_empty_output(self, mock_run):
        """Test calculating coverage with empty output."""
        mock_run.side_effect = [
            Mock(stdout="", stderr="", returncode=0),  # pytest run
            Mock(stdout="", stderr="", returncode=0),  # coverage json
        ]

        calculator = CoverageCalculator()
        result = calculator.calculate(Path("/test/project"))

        assert result.line_coverage_percent == 0.0
        assert result.branch_coverage_percent == 0.0
        assert result.uncovered_lines == 0

    @patch("src.quality.coverage.subprocess.run")
    def test_calculate_invalid_json(self, mock_run):
        """Test handling of invalid JSON output."""
        mock_run.side_effect = [
            Mock(stdout="", stderr="", returncode=0),  # pytest run
            Mock(stdout="invalid json", stderr="", returncode=0),  # coverage json
        ]

        calculator = CoverageCalculator()
        with pytest.raises(CoverageError) as exc_info:
            calculator.calculate(Path("/test/project"))

        assert "json" in str(exc_info.value).lower()

    @patch("src.quality.coverage.subprocess.run")
    def test_calculate_timeout(self, mock_run):
        """Test handling of timeout."""
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired("coverage", 600)

        calculator = CoverageCalculator()
        with pytest.raises(CoverageError) as exc_info:
            calculator.calculate(Path("/test/project"))

        assert "timed out" in str(exc_info.value).lower()

    @patch("src.quality.coverage.subprocess.run")
    def test_calculate_not_installed(self, mock_run):
        """Test handling when coverage is not installed."""
        mock_run.side_effect = FileNotFoundError()

        calculator = CoverageCalculator()
        with pytest.raises(CoverageError) as exc_info:
            calculator.calculate(Path("/test/project"))

        assert "not installed" in str(exc_info.value).lower()

    @patch("src.quality.coverage.subprocess.run")
    def test_get_coverage_report_success(self, mock_run):
        """Test getting detailed coverage report."""
        coverage_output = json.dumps({
            "files": {
                "src/main.py": {
                    "covered_lines": [1, 2, 3],
                    "missing_lines": [4, 5],
                },
            },
            "totals": {"percent_covered": 60.0},
        })

        mock_run.return_value = Mock(
            stdout=coverage_output,
            stderr="",
            returncode=0,
        )

        calculator = CoverageCalculator()
        report = calculator.get_coverage_report(Path("/test/project"))

        assert "files" in report
        assert "src/main.py" in report["files"]

    @patch("src.quality.coverage.subprocess.run")
    def test_get_coverage_report_empty(self, mock_run):
        """Test getting coverage report with no data."""
        mock_run.return_value = Mock(
            stdout="",
            stderr="",
            returncode=0,
        )

        calculator = CoverageCalculator()
        report = calculator.get_coverage_report(Path("/test/project"))

        assert report == {}

    @patch("src.quality.coverage.subprocess.run")
    def test_get_coverage_report_error(self, mock_run):
        """Test getting coverage report with error."""
        import subprocess
        mock_run.side_effect = subprocess.SubprocessError()

        calculator = CoverageCalculator()
        report = calculator.get_coverage_report(Path("/test/project"))

        # Should return empty dict on error
        assert report == {}

    def test_check_coverage_threshold_pass(self):
        """Test coverage threshold check - passing."""
        calculator = CoverageCalculator()
        calculator.min_coverage = 80.0

        coverage = CoverageResults(
            line_coverage_percent=85.0,
            branch_coverage_percent=70.0,
            uncovered_lines=100,
        )

        assert calculator.check_coverage_threshold(coverage) is True

    def test_check_coverage_threshold_fail(self):
        """Test coverage threshold check - failing."""
        calculator = CoverageCalculator()
        calculator.min_coverage = 80.0

        coverage = CoverageResults(
            line_coverage_percent=75.0,
            branch_coverage_percent=70.0,
            uncovered_lines=200,
        )

        assert calculator.check_coverage_threshold(coverage) is False

    def test_check_coverage_threshold_exact(self):
        """Test coverage threshold check - exactly at threshold."""
        calculator = CoverageCalculator()
        calculator.min_coverage = 80.0

        coverage = CoverageResults(
            line_coverage_percent=80.0,
            branch_coverage_percent=70.0,
            uncovered_lines=150,
        )

        assert calculator.check_coverage_threshold(coverage) is True

    @patch.object(CoverageCalculator, "get_coverage_report")
    def test_get_uncovered_lines_success(self, mock_report):
        """Test getting uncovered lines for a file."""
        mock_report.return_value = {
            "files": {
                "src/main.py": {
                    "missing_lines": [10, 15, 20],
                },
            },
        }

        calculator = CoverageCalculator()
        with patch.object(Path, "relative_to", return_value=Path("src/main.py")):
            uncovered = calculator.get_uncovered_lines(
                Path("/test/project"),
                Path("/test/project/src/main.py"),
            )

        assert uncovered == [10, 15, 20]

    @patch.object(CoverageCalculator, "get_coverage_report")
    def test_get_uncovered_lines_file_not_found(self, mock_report):
        """Test getting uncovered lines for non-existent file."""
        mock_report.return_value = {
            "files": {
                "src/main.py": {"missing_lines": [10, 15]},
            },
        }

        calculator = CoverageCalculator()
        with patch.object(Path, "relative_to", return_value=Path("src/other.py")):
            uncovered = calculator.get_uncovered_lines(
                Path("/test/project"),
                Path("/test/project/src/other.py"),
            )

        # Should return empty list for file not in report
        assert uncovered == []


class TestCoverageResults:
    """Tests for CoverageResults model."""

    def test_default_values(self):
        """Test default values for CoverageResults."""
        results = CoverageResults()

        assert results.line_coverage_percent == 0.0
        assert results.branch_coverage_percent == 0.0
        assert results.uncovered_lines == 0
        assert results.by_test_type is not None

    def test_with_values(self):
        """Test CoverageResults with values."""
        results = CoverageResults(
            line_coverage_percent=85.5,
            branch_coverage_percent=75.0,
            uncovered_lines=150,
            by_test_type=CoverageByType(
                unit=60.0,
                integration=20.0,
                e2e=5.5,
            ),
        )

        assert results.line_coverage_percent == 85.5
        assert results.branch_coverage_percent == 75.0
        assert results.uncovered_lines == 150
        assert results.by_test_type.unit == 60.0
        assert results.by_test_type.integration == 20.0
        assert results.by_test_type.e2e == 5.5


class TestGetCoverageCalculator:
    """Tests for get_coverage_calculator function."""

    def test_get_coverage_calculator(self):
        """Test getting coverage calculator instance."""
        calculator = get_coverage_calculator()
        assert isinstance(calculator, CoverageCalculator)
