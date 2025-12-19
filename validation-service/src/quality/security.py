"""Security scanner service.

This module provides security scanning using Bandit for Python code.
It detects security vulnerabilities and returns structured results.
"""

import json
import logging
import subprocess
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.core.exceptions import SecurityScanError
from src.quality.models import SecurityResults, Severity, Vulnerability


logger = logging.getLogger(__name__)


class SecurityScanner:
    """Service for running security scans."""

    def __init__(self):
        """Initialize the security scanner."""
        self.settings = get_settings()

    def scan(self, project_path: Path) -> SecurityResults:
        """Run security scan on Python files.

        Args:
            project_path: Path to project root

        Returns:
            SecurityResults with vulnerabilities found

        Raises:
            SecurityScanError: If Bandit fails to execute
        """
        logger.info(f"Running security scan on {project_path}")

        try:
            # Run bandit with JSON output
            result = subprocess.run(
                [
                    "bandit",
                    "-r",
                    str(project_path),
                    "-f", "json",
                    "-q",  # Quiet mode
                ],
                capture_output=True,
                text=True,
                timeout=300,
            )

            vulnerabilities = []
            critical_count = 0
            high_count = 0
            medium_count = 0
            low_count = 0

            # Bandit returns non-zero if issues found, but still produces valid output
            if result.stdout:
                try:
                    bandit_results = json.loads(result.stdout)
                    for issue in bandit_results.get("results", []):
                        severity = self._map_bandit_severity(issue.get("issue_severity", ""))

                        vuln = Vulnerability(
                            package=issue.get("filename", ""),
                            version="",  # Not applicable for code issues
                            severity=severity,
                            cve_id=None,
                            description=f"[{issue.get('test_id', '')}] {issue.get('issue_text', '')}",
                            fixed_version=None,
                        )
                        vulnerabilities.append(vuln)

                        if severity == Severity.CRITICAL:
                            critical_count += 1
                        elif severity == Severity.HIGH:
                            high_count += 1
                        elif severity == Severity.MEDIUM:
                            medium_count += 1
                        else:
                            low_count += 1

                except json.JSONDecodeError:
                    logger.warning("Failed to parse Bandit JSON output")

            return SecurityResults(
                vulnerabilities=vulnerabilities,
                critical_count=critical_count,
                high_count=high_count,
                medium_count=medium_count,
                low_count=low_count,
            )

        except subprocess.TimeoutExpired:
            raise SecurityScanError("bandit", "Security scan timed out after 300s")
        except FileNotFoundError:
            raise SecurityScanError("bandit", "Bandit not installed. Run: pip install bandit")
        except subprocess.SubprocessError as e:
            raise SecurityScanError("bandit", str(e))

    def _map_bandit_severity(self, bandit_severity: str) -> Severity:
        """Map Bandit severity to our severity levels.

        Args:
            bandit_severity: Bandit severity string (HIGH, MEDIUM, LOW)

        Returns:
            Severity level
        """
        severity_map = {
            "HIGH": Severity.HIGH,
            "MEDIUM": Severity.MEDIUM,
            "LOW": Severity.LOW,
        }
        return severity_map.get(bandit_severity.upper(), Severity.LOW)

    def scan_dependencies(self, project_path: Path) -> list[Vulnerability]:
        """Scan dependencies for known vulnerabilities using Safety.

        Args:
            project_path: Path to project root

        Returns:
            List of dependency vulnerabilities

        Raises:
            SecurityScanError: If Safety fails to execute
        """
        logger.info(f"Scanning dependencies for {project_path}")

        vulnerabilities = []
        requirements_file = project_path / "requirements.txt"

        if not requirements_file.exists():
            logger.warning("No requirements.txt found, skipping dependency scan")
            return []

        try:
            result = subprocess.run(
                [
                    "safety",
                    "check",
                    "-r", str(requirements_file),
                    "--json",
                ],
                capture_output=True,
                text=True,
                timeout=120,
            )

            if result.stdout:
                try:
                    safety_results = json.loads(result.stdout)
                    for vuln in safety_results:
                        if isinstance(vuln, list) and len(vuln) >= 5:
                            vulnerabilities.append(
                                Vulnerability(
                                    package=vuln[0],
                                    version=vuln[2],
                                    severity=Severity.HIGH,  # Safety doesn't provide severity
                                    cve_id=vuln[4] if len(vuln) > 4 else None,
                                    description=vuln[3] if len(vuln) > 3 else "",
                                    fixed_version=None,
                                )
                            )
                except json.JSONDecodeError:
                    logger.warning("Failed to parse Safety JSON output")

            return vulnerabilities

        except subprocess.TimeoutExpired:
            raise SecurityScanError("safety", "Dependency scan timed out after 120s")
        except FileNotFoundError:
            raise SecurityScanError("safety", "Safety not installed. Run: pip install safety")
        except subprocess.SubprocessError as e:
            raise SecurityScanError("safety", str(e))

    def scan_file(self, file_path: Path) -> list[Vulnerability]:
        """Scan a single file for security issues.

        Args:
            file_path: Path to file to scan

        Returns:
            List of vulnerabilities found
        """
        if not file_path.exists():
            return []

        try:
            result = subprocess.run(
                ["bandit", str(file_path), "-f", "json", "-q"],
                capture_output=True,
                text=True,
                timeout=60,
            )

            vulnerabilities = []
            if result.stdout:
                try:
                    bandit_results = json.loads(result.stdout)
                    for issue in bandit_results.get("results", []):
                        vulnerabilities.append(
                            Vulnerability(
                                package=issue.get("filename", ""),
                                version="",
                                severity=self._map_bandit_severity(
                                    issue.get("issue_severity", "")
                                ),
                                cve_id=None,
                                description=issue.get("issue_text", ""),
                                fixed_version=None,
                            )
                        )
                except json.JSONDecodeError:
                    pass

            return vulnerabilities

        except (subprocess.SubprocessError, FileNotFoundError):
            return []


def get_security_scanner() -> SecurityScanner:
    """Get security scanner instance.

    Returns:
        SecurityScanner instance
    """
    return SecurityScanner()
