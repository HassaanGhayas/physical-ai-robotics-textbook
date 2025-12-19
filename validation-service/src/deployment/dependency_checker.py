"""Dependency checker service.

This module provides dependency validation with conflict detection
and security vulnerability scanning.
"""

import json
import logging
import re
import subprocess
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.deployment.models import (
    ConfigCheckStatus,
    DependencyConflict,
    OutdatedPackage,
    SeverityLevel,
    DependencyValidationResult,
)


logger = logging.getLogger(__name__)


class DependencyChecker:
    """Service for checking project dependencies."""

    def __init__(self):
        """Initialize the dependency checker."""
        self.settings = get_settings()

    def check(
        self,
        project_path: Path,
        check_outdated: bool = True,
        check_vulnerabilities: bool = True,
    ) -> DependencyValidationResult:
        """Check project dependencies.

        Args:
            project_path: Path to project root
            check_outdated: Whether to check for outdated packages
            check_vulnerabilities: Whether to scan for vulnerabilities

        Returns:
            DependencyValidationResult
        """
        result = DependencyValidationResult()

        # Detect package manager
        manager = self._detect_package_manager(project_path)

        if manager == "pip":
            result = self._check_python_dependencies(
                project_path, check_outdated, check_vulnerabilities
            )
        elif manager == "npm":
            result = self._check_npm_dependencies(
                project_path, check_outdated, check_vulnerabilities
            )
        elif manager == "yarn":
            result = self._check_yarn_dependencies(
                project_path, check_outdated, check_vulnerabilities
            )
        else:
            result.warnings.append("No recognized package manager found")

        # Determine overall status
        if result.conflicts or result.security_vulnerabilities > 0:
            result.status = ConfigCheckStatus.INVALID
        elif result.outdated_packages:
            result.status = ConfigCheckStatus.WARNING
        else:
            result.status = ConfigCheckStatus.VALID

        return result

    def _detect_package_manager(self, project_path: Path) -> Optional[str]:
        """Detect which package manager the project uses.

        Args:
            project_path: Path to project root

        Returns:
            Package manager name or None
        """
        if (project_path / "requirements.txt").exists():
            return "pip"
        if (project_path / "pyproject.toml").exists():
            return "pip"
        if (project_path / "package-lock.json").exists():
            return "npm"
        if (project_path / "yarn.lock").exists():
            return "yarn"
        if (project_path / "package.json").exists():
            return "npm"  # Default to npm if package.json exists

        return None

    def _check_python_dependencies(
        self,
        project_path: Path,
        check_outdated: bool,
        check_vulnerabilities: bool,
    ) -> DependencyValidationResult:
        """Check Python dependencies.

        Args:
            project_path: Path to project root
            check_outdated: Whether to check for outdated packages
            check_vulnerabilities: Whether to scan for vulnerabilities

        Returns:
            DependencyValidationResult
        """
        result = DependencyValidationResult()

        # Parse requirements
        requirements = self._parse_requirements(project_path)
        result.total_dependencies = len(requirements)

        # Check for conflicts
        conflicts = self._find_python_conflicts(requirements)
        result.conflicts.extend(conflicts)

        # Check for outdated packages
        if check_outdated:
            try:
                outdated = self._check_pip_outdated(project_path)
                result.outdated_packages.extend(outdated)
            except Exception as e:
                logger.warning(f"Could not check outdated packages: {e}")
                result.warnings.append(f"Could not check outdated packages: {e}")

        # Check for vulnerabilities
        if check_vulnerabilities:
            vuln_count = self._check_python_vulnerabilities(project_path)
            result.security_vulnerabilities = vuln_count

        return result

    def _parse_requirements(self, project_path: Path) -> dict[str, str]:
        """Parse requirements from requirements.txt or pyproject.toml.

        Args:
            project_path: Path to project root

        Returns:
            Dict mapping package name to version constraint
        """
        requirements = {}

        # Check requirements.txt
        req_file = project_path / "requirements.txt"
        if req_file.exists():
            content = req_file.read_text()
            for line in content.split("\n"):
                line = line.strip()
                if not line or line.startswith("#") or line.startswith("-"):
                    continue

                # Parse package spec
                match = re.match(r"([a-zA-Z0-9_-]+)([<>=!~].*)?", line)
                if match:
                    name = match.group(1).lower()
                    version = match.group(2) or "*"
                    requirements[name] = version

        return requirements

    def _find_python_conflicts(
        self, requirements: dict[str, str]
    ) -> list[DependencyConflict]:
        """Find conflicts in Python dependencies.

        Args:
            requirements: Dict of package names to versions

        Returns:
            List of DependencyConflict
        """
        conflicts = []

        # This is a simplified conflict detection
        # In production, you'd use pip's resolver or pipdeptree
        try:
            result = subprocess.run(
                ["pip", "check"],
                capture_output=True,
                text=True,
                timeout=60,
            )

            if result.returncode != 0:
                # Parse pip check output
                for line in result.stdout.split("\n"):
                    if "requires" in line and "which is not installed" in line:
                        conflicts.append(DependencyConflict(
                            package=line.split()[0],
                            required_by=[],
                            versions_required=[],
                            conflict_type="missing_dependency",
                            severity=SeverityLevel.HIGH,
                            resolution="Install missing dependency",
                        ))
                    elif "has requirement" in line:
                        conflicts.append(DependencyConflict(
                            package=line.split()[0],
                            required_by=[],
                            versions_required=[],
                            conflict_type="version_conflict",
                            severity=SeverityLevel.MEDIUM,
                            resolution="Resolve version conflict",
                        ))

        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            logger.warning(f"Could not run pip check: {e}")

        return conflicts

    def _check_pip_outdated(self, project_path: Path) -> list[OutdatedPackage]:
        """Check for outdated pip packages.

        Args:
            project_path: Path to project root

        Returns:
            List of OutdatedPackage
        """
        outdated = []

        try:
            result = subprocess.run(
                ["pip", "list", "--outdated", "--format=json"],
                capture_output=True,
                text=True,
                timeout=120,
                cwd=project_path,
            )

            if result.returncode == 0:
                packages = json.loads(result.stdout)
                for pkg in packages:
                    outdated.append(OutdatedPackage(
                        name=pkg["name"],
                        current_version=pkg["version"],
                        latest_version=pkg["latest_version"],
                        severity=SeverityLevel.LOW,
                    ))

        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as e:
            logger.warning(f"Could not check outdated packages: {e}")

        return outdated

    def _check_python_vulnerabilities(self, project_path: Path) -> int:
        """Check for security vulnerabilities in Python dependencies.

        Args:
            project_path: Path to project root

        Returns:
            Number of vulnerabilities found
        """
        vuln_count = 0

        # Try pip-audit if available
        try:
            result = subprocess.run(
                ["pip-audit", "--format=json"],
                capture_output=True,
                text=True,
                timeout=120,
                cwd=project_path,
            )

            if result.returncode != 0 and result.stdout:
                data = json.loads(result.stdout)
                vuln_count = len(data.get("vulnerabilities", []))

        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError):
            # pip-audit not available, try safety
            try:
                result = subprocess.run(
                    ["safety", "check", "--json"],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    cwd=project_path,
                )

                if result.stdout:
                    data = json.loads(result.stdout)
                    if isinstance(data, list):
                        vuln_count = len(data)

            except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError):
                logger.warning("Neither pip-audit nor safety available for vulnerability scanning")

        return vuln_count

    def _check_npm_dependencies(
        self,
        project_path: Path,
        check_outdated: bool,
        check_vulnerabilities: bool,
    ) -> DependencyValidationResult:
        """Check npm dependencies.

        Args:
            project_path: Path to project root
            check_outdated: Whether to check for outdated packages
            check_vulnerabilities: Whether to scan for vulnerabilities

        Returns:
            DependencyValidationResult
        """
        result = DependencyValidationResult()

        # Parse package.json
        package_json = project_path / "package.json"
        if package_json.exists():
            data = json.loads(package_json.read_text())
            deps = data.get("dependencies", {})
            dev_deps = data.get("devDependencies", {})
            result.total_dependencies = len(deps) + len(dev_deps)

        # Check for outdated
        if check_outdated:
            try:
                out_result = subprocess.run(
                    ["npm", "outdated", "--json"],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    cwd=project_path,
                )

                if out_result.stdout:
                    data = json.loads(out_result.stdout)
                    for name, info in data.items():
                        result.outdated_packages.append(OutdatedPackage(
                            name=name,
                            current_version=info.get("current", "unknown"),
                            latest_version=info.get("latest", "unknown"),
                            severity=SeverityLevel.LOW,
                        ))

            except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as e:
                logger.warning(f"Could not check npm outdated: {e}")

        # Check for vulnerabilities with npm audit
        if check_vulnerabilities:
            try:
                audit_result = subprocess.run(
                    ["npm", "audit", "--json"],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    cwd=project_path,
                )

                if audit_result.stdout:
                    data = json.loads(audit_result.stdout)
                    metadata = data.get("metadata", {})
                    vuln_info = metadata.get("vulnerabilities", {})
                    result.security_vulnerabilities = sum(vuln_info.values())

            except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as e:
                logger.warning(f"Could not run npm audit: {e}")

        return result

    def _check_yarn_dependencies(
        self,
        project_path: Path,
        check_outdated: bool,
        check_vulnerabilities: bool,
    ) -> DependencyValidationResult:
        """Check yarn dependencies.

        Args:
            project_path: Path to project root
            check_outdated: Whether to check for outdated packages
            check_vulnerabilities: Whether to scan for vulnerabilities

        Returns:
            DependencyValidationResult
        """
        # Similar to npm, yarn has yarn outdated and yarn audit
        result = self._check_npm_dependencies(
            project_path, check_outdated, check_vulnerabilities
        )

        # Replace npm commands with yarn if needed
        if check_vulnerabilities:
            try:
                audit_result = subprocess.run(
                    ["yarn", "audit", "--json"],
                    capture_output=True,
                    text=True,
                    timeout=120,
                    cwd=project_path,
                )

                if audit_result.returncode != 0 and audit_result.stdout:
                    # Parse yarn audit output
                    for line in audit_result.stdout.split("\n"):
                        if line:
                            try:
                                data = json.loads(line)
                                if data.get("type") == "auditSummary":
                                    vulns = data.get("data", {}).get("vulnerabilities", {})
                                    result.security_vulnerabilities = sum(vulns.values())
                            except json.JSONDecodeError:
                                continue

            except (subprocess.TimeoutExpired, FileNotFoundError):
                pass

        return result


def get_dependency_checker() -> DependencyChecker:
    """Get dependency checker instance.

    Returns:
        DependencyChecker instance
    """
    return DependencyChecker()
