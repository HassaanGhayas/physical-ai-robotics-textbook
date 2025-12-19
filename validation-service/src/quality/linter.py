"""Linter service for code style validation.

This module provides linting functionality using Ruff for Python code.
It detects style violations and returns structured results.
"""

import json
import logging
import subprocess
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.core.exceptions import LinterError
from src.quality.models import LintingResults, Severity, Violation


logger = logging.getLogger(__name__)


class LinterService:
    """Service for running code linters."""

    def __init__(self):
        """Initialize the linter service."""
        self.settings = get_settings()

    def run_ruff(self, project_path: Path) -> LintingResults:
        """Run Ruff linter on Python files.

        Args:
            project_path: Path to project root

        Returns:
            LintingResults with violations found

        Raises:
            LinterError: If Ruff fails to execute
        """
        logger.info(f"Running Ruff linter on {project_path}")

        try:
            # Run ruff check with JSON output
            result = subprocess.run(
                [
                    "ruff",
                    "check",
                    str(project_path),
                    "--output-format=json",
                    "--quiet",
                ],
                capture_output=True,
                text=True,
                timeout=300,  # 5 minute timeout
            )

            violations = []
            error_count = 0
            warning_count = 0

            if result.stdout:
                try:
                    ruff_results = json.loads(result.stdout)
                    for item in ruff_results:
                        severity = self._map_ruff_severity(item.get("code", ""))
                        violation = Violation(
                            file=item.get("filename", ""),
                            line=item.get("location", {}).get("row", 1),
                            column=item.get("location", {}).get("column"),
                            rule=item.get("code", ""),
                            severity=severity,
                            message=item.get("message", ""),
                        )
                        violations.append(violation)

                        if severity == Severity.ERROR:
                            error_count += 1
                        else:
                            warning_count += 1
                except json.JSONDecodeError:
                    logger.warning("Failed to parse Ruff JSON output")

            return LintingResults(
                violations=violations,
                total_violations=len(violations),
                error_count=error_count,
                warning_count=warning_count,
            )

        except subprocess.TimeoutExpired:
            raise LinterError("ruff", "Linting timed out after 300 seconds")
        except FileNotFoundError:
            raise LinterError("ruff", "Ruff not installed. Run: pip install ruff")
        except subprocess.SubprocessError as e:
            raise LinterError("ruff", str(e))

    def _map_ruff_severity(self, rule_code: str) -> Severity:
        """Map Ruff rule codes to severity levels.

        Args:
            rule_code: Ruff rule code (e.g., E501, W503)

        Returns:
            Severity level
        """
        # Error categories (typically errors)
        if rule_code.startswith(("E", "F")):
            return Severity.ERROR
        # Warning categories
        if rule_code.startswith(("W", "C", "B")):
            return Severity.WARNING
        # Info categories
        if rule_code.startswith(("I", "D")):
            return Severity.INFO
        return Severity.WARNING

    def check_file(self, file_path: Path) -> list[Violation]:
        """Lint a single file.

        Args:
            file_path: Path to file to lint

        Returns:
            List of violations found
        """
        if not file_path.exists():
            return []

        results = self.run_ruff(file_path.parent)
        # Filter to only violations in this specific file
        file_str = str(file_path)
        return [v for v in results.violations if v.file == file_str]


def get_linter_service() -> LinterService:
    """Get linter service instance.

    Returns:
        LinterService instance
    """
    return LinterService()
