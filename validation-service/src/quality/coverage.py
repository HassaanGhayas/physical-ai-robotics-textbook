"""Coverage calculator service.

This module provides test coverage measurement using coverage.py.
It calculates line and branch coverage for Python projects.
"""

import json
import logging
import subprocess
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.core.exceptions import CoverageError
from src.quality.models import CoverageByType, CoverageResults


logger = logging.getLogger(__name__)


class CoverageCalculator:
    """Service for calculating test coverage."""

    def __init__(self):
        """Initialize the coverage calculator."""
        self.settings = get_settings()
        self.min_coverage = self.settings.quality.min_coverage_percentage

    def calculate(self, project_path: Path) -> CoverageResults:
        """Calculate test coverage for project.

        This runs the test suite with coverage measurement and
        returns the coverage metrics.

        Args:
            project_path: Path to project root

        Returns:
            CoverageResults with coverage metrics

        Raises:
            CoverageError: If coverage calculation fails
        """
        logger.info(f"Calculating coverage for {project_path}")

        try:
            # First, run tests with coverage
            run_result = subprocess.run(
                [
                    "coverage",
                    "run",
                    "-m",
                    "pytest",
                    "--quiet",
                ],
                cwd=project_path,
                capture_output=True,
                text=True,
                timeout=600,  # 10 minute timeout for tests
            )

            # Generate JSON report
            report_result = subprocess.run(
                ["coverage", "json", "-o", "-"],
                cwd=project_path,
                capture_output=True,
                text=True,
                timeout=60,
            )

            if not report_result.stdout:
                return CoverageResults(
                    line_coverage_percent=0.0,
                    branch_coverage_percent=0.0,
                    uncovered_lines=0,
                    by_test_type=CoverageByType(),
                )

            coverage_data = json.loads(report_result.stdout)
            totals = coverage_data.get("totals", {})

            line_coverage = totals.get("percent_covered", 0.0)
            branch_coverage = totals.get("percent_covered_branches", 0.0)
            uncovered = totals.get("missing_lines", 0)

            return CoverageResults(
                line_coverage_percent=round(line_coverage, 2),
                branch_coverage_percent=round(branch_coverage, 2) if branch_coverage else 0.0,
                uncovered_lines=uncovered,
                by_test_type=CoverageByType(
                    unit=line_coverage,  # Default all to unit for now
                    integration=0.0,
                    e2e=0.0,
                ),
            )

        except json.JSONDecodeError:
            raise CoverageError("Failed to parse coverage JSON output")
        except subprocess.TimeoutExpired:
            raise CoverageError("Coverage calculation timed out")
        except FileNotFoundError:
            raise CoverageError("Coverage not installed. Run: pip install coverage pytest")
        except subprocess.SubprocessError as e:
            raise CoverageError(str(e))

    def get_coverage_report(self, project_path: Path) -> dict:
        """Get detailed coverage report.

        Args:
            project_path: Path to project root

        Returns:
            Dictionary with per-file coverage data
        """
        try:
            result = subprocess.run(
                ["coverage", "json", "-o", "-"],
                cwd=project_path,
                capture_output=True,
                text=True,
                timeout=60,
            )

            if result.stdout:
                return json.loads(result.stdout)
            return {}

        except (json.JSONDecodeError, subprocess.SubprocessError, FileNotFoundError):
            return {}

    def check_coverage_threshold(self, coverage: CoverageResults) -> bool:
        """Check if coverage meets minimum threshold.

        Args:
            coverage: Coverage results to check

        Returns:
            True if coverage meets or exceeds minimum threshold
        """
        return coverage.line_coverage_percent >= self.min_coverage

    def get_uncovered_lines(self, project_path: Path, file_path: Path) -> list[int]:
        """Get list of uncovered line numbers for a specific file.

        Args:
            project_path: Path to project root
            file_path: Path to specific file

        Returns:
            List of uncovered line numbers
        """
        report = self.get_coverage_report(project_path)
        files = report.get("files", {})

        relative_path = str(file_path.relative_to(project_path))
        file_data = files.get(relative_path, {})

        return file_data.get("missing_lines", [])


def get_coverage_calculator() -> CoverageCalculator:
    """Get coverage calculator instance.

    Returns:
        CoverageCalculator instance
    """
    return CoverageCalculator()
