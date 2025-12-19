"""Security scanner service for deployment validation.

This module provides security scanning for deployment configurations,
including vulnerability detection and security best practices checks.
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
    SecretRisk,
    SeverityLevel,
    ConfigurationValidationResult,
)


logger = logging.getLogger(__name__)


# Security-related patterns to detect
SECURITY_PATTERNS = {
    "hardcoded_ip": (
        r"\b(?:(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(?:25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\b",
        "Hardcoded IP Address",
        SeverityLevel.LOW,
    ),
    "debug_enabled": (
        r"(?i)(debug|DEBUG)\s*[=:]\s*(true|True|1|on|ON)",
        "Debug Mode Enabled",
        SeverityLevel.MEDIUM,
    ),
    "insecure_protocol": (
        r"http://(?!localhost|127\.0\.0\.1)",
        "Insecure HTTP Protocol",
        SeverityLevel.MEDIUM,
    ),
    "eval_usage": (
        r"\beval\s*\(",
        "Unsafe eval() Usage",
        SeverityLevel.HIGH,
    ),
    "exec_usage": (
        r"\bexec\s*\(",
        "Unsafe exec() Usage",
        SeverityLevel.HIGH,
    ),
    "sql_injection_risk": (
        r'f["\'].*SELECT.*\{',
        "Potential SQL Injection",
        SeverityLevel.CRITICAL,
    ),
    "shell_injection_risk": (
        r"subprocess\.(?:call|run|Popen)\([^)]*shell\s*=\s*True",
        "Potential Shell Injection",
        SeverityLevel.HIGH,
    ),
    "pickle_usage": (
        r"pickle\.(?:load|loads)\(",
        "Unsafe Pickle Usage",
        SeverityLevel.HIGH,
    ),
    "yaml_unsafe_load": (
        r"yaml\.(?:load|unsafe_load)\(",
        "Unsafe YAML Load",
        SeverityLevel.HIGH,
    ),
    "jwt_no_verify": (
        r"jwt\.decode\([^)]*verify\s*=\s*False",
        "JWT Verification Disabled",
        SeverityLevel.CRITICAL,
    ),
}


class SecurityScanner:
    """Service for scanning deployment configurations for security issues."""

    def __init__(self):
        """Initialize the security scanner."""
        self.settings = get_settings()

    def scan(
        self,
        project_path: Path,
        include_bandit: bool = True,
        include_semgrep: bool = False,
    ) -> ConfigurationValidationResult:
        """Scan project for security issues.

        Args:
            project_path: Path to project root
            include_bandit: Run Bandit security scanner
            include_semgrep: Run Semgrep security scanner

        Returns:
            ConfigurationValidationResult with security findings
        """
        result = ConfigurationValidationResult()

        # Pattern-based scanning
        pattern_findings = self._scan_patterns(project_path)
        result.secret_risks.extend(pattern_findings)

        # Run Bandit if requested
        if include_bandit:
            bandit_findings = self._run_bandit(project_path)
            result.secret_risks.extend(bandit_findings)

        # Run Semgrep if requested
        if include_semgrep:
            semgrep_findings = self._run_semgrep(project_path)
            result.secret_risks.extend(semgrep_findings)

        # Check security configurations
        config_checks = self._check_security_configs(project_path)
        result.checks.extend(config_checks)

        # Determine status
        critical_count = sum(
            1 for s in result.secret_risks if s.severity == SeverityLevel.CRITICAL
        )
        high_count = sum(
            1 for s in result.secret_risks if s.severity == SeverityLevel.HIGH
        )

        if critical_count > 0:
            result.status = ConfigCheckStatus.INVALID
        elif high_count > 0:
            result.status = ConfigCheckStatus.WARNING
        else:
            result.status = ConfigCheckStatus.VALID

        return result

    def _scan_patterns(self, project_path: Path) -> list[SecretRisk]:
        """Scan project files for security patterns.

        Args:
            project_path: Path to project root

        Returns:
            List of SecretRisk findings
        """
        findings = []

        # File patterns to scan
        patterns = ["*.py", "*.js", "*.ts", "*.jsx", "*.tsx"]

        # Directories to skip
        skip_dirs = {".git", "node_modules", "__pycache__", "venv", ".venv", "dist", "build"}

        for pattern in patterns:
            for file_path in project_path.rglob(pattern):
                if any(skip_dir in file_path.parts for skip_dir in skip_dirs):
                    continue

                file_findings = self._scan_file_patterns(file_path)
                findings.extend(file_findings)

        return findings

    def _scan_file_patterns(self, file_path: Path) -> list[SecretRisk]:
        """Scan a single file for security patterns.

        Args:
            file_path: Path to file

        Returns:
            List of SecretRisk findings
        """
        findings = []

        try:
            content = file_path.read_text(errors="ignore")
            lines = content.split("\n")

            for line_num, line in enumerate(lines, 1):
                for pattern_name, (pattern, description, severity) in SECURITY_PATTERNS.items():
                    if re.search(pattern, line):
                        findings.append(SecretRisk(
                            file_path=str(file_path),
                            line_number=line_num,
                            secret_type=pattern_name,
                            severity=severity,
                            pattern_matched=description,
                            recommendation=self._get_recommendation(pattern_name),
                        ))

        except Exception as e:
            logger.debug(f"Error scanning file {file_path}: {e}")

        return findings

    def _get_recommendation(self, pattern_name: str) -> str:
        """Get remediation recommendation for a pattern.

        Args:
            pattern_name: Name of the detected pattern

        Returns:
            Recommendation string
        """
        recommendations = {
            "hardcoded_ip": "Use environment variables or configuration for IP addresses",
            "debug_enabled": "Disable debug mode in production",
            "insecure_protocol": "Use HTTPS instead of HTTP",
            "eval_usage": "Avoid eval(), use safer alternatives like ast.literal_eval()",
            "exec_usage": "Avoid exec(), refactor code to avoid dynamic execution",
            "sql_injection_risk": "Use parameterized queries instead of string formatting",
            "shell_injection_risk": "Avoid shell=True, use list of arguments instead",
            "pickle_usage": "Use safer serialization like JSON for untrusted data",
            "yaml_unsafe_load": "Use yaml.safe_load() instead",
            "jwt_no_verify": "Enable JWT signature verification",
        }
        return recommendations.get(pattern_name, "Review and fix the security issue")

    def _run_bandit(self, project_path: Path) -> list[SecretRisk]:
        """Run Bandit security scanner.

        Args:
            project_path: Path to project root

        Returns:
            List of SecretRisk findings
        """
        findings = []

        try:
            result = subprocess.run(
                ["bandit", "-r", str(project_path), "-f", "json", "-q"],
                capture_output=True,
                text=True,
                timeout=300,
            )

            if result.stdout:
                data = json.loads(result.stdout)
                for issue in data.get("results", []):
                    severity_map = {
                        "HIGH": SeverityLevel.HIGH,
                        "MEDIUM": SeverityLevel.MEDIUM,
                        "LOW": SeverityLevel.LOW,
                    }
                    findings.append(SecretRisk(
                        file_path=issue.get("filename", "unknown"),
                        line_number=issue.get("line_number", 0),
                        secret_type=issue.get("test_id", "bandit"),
                        severity=severity_map.get(issue.get("issue_severity"), SeverityLevel.MEDIUM),
                        pattern_matched=issue.get("issue_text", "Security issue"),
                        recommendation=issue.get("more_info", "Review security issue"),
                    ))

        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as e:
            logger.debug(f"Could not run Bandit: {e}")

        return findings

    def _run_semgrep(self, project_path: Path) -> list[SecretRisk]:
        """Run Semgrep security scanner.

        Args:
            project_path: Path to project root

        Returns:
            List of SecretRisk findings
        """
        findings = []

        try:
            result = subprocess.run(
                [
                    "semgrep",
                    "--config=auto",
                    "--json",
                    str(project_path),
                ],
                capture_output=True,
                text=True,
                timeout=300,
            )

            if result.stdout:
                data = json.loads(result.stdout)
                for issue in data.get("results", []):
                    severity_map = {
                        "ERROR": SeverityLevel.HIGH,
                        "WARNING": SeverityLevel.MEDIUM,
                        "INFO": SeverityLevel.LOW,
                    }
                    findings.append(SecretRisk(
                        file_path=issue.get("path", "unknown"),
                        line_number=issue.get("start", {}).get("line", 0),
                        secret_type=issue.get("check_id", "semgrep"),
                        severity=severity_map.get(issue.get("extra", {}).get("severity"), SeverityLevel.MEDIUM),
                        pattern_matched=issue.get("extra", {}).get("message", "Security issue"),
                        recommendation="Review and fix the security issue",
                    ))

        except (subprocess.TimeoutExpired, FileNotFoundError, json.JSONDecodeError) as e:
            logger.debug(f"Could not run Semgrep: {e}")

        return findings

    def _check_security_configs(self, project_path: Path) -> list:
        """Check security-related configuration files.

        Args:
            project_path: Path to project root

        Returns:
            List of ConfigCheck results
        """
        from src.deployment.models import ConfigCheck

        checks = []

        # Check for .gitignore
        gitignore = project_path / ".gitignore"
        if gitignore.exists():
            content = gitignore.read_text()
            required_ignores = [".env", "*.pem", "*.key", "credentials"]
            missing = [ig for ig in required_ignores if ig not in content]
            if missing:
                checks.append(ConfigCheck(
                    name=".gitignore completeness",
                    status=ConfigCheckStatus.WARNING,
                    error_message=f"Missing security-related ignores: {', '.join(missing)}",
                    location=str(gitignore),
                ))
            else:
                checks.append(ConfigCheck(
                    name=".gitignore completeness",
                    status=ConfigCheckStatus.VALID,
                    location=str(gitignore),
                ))
        else:
            checks.append(ConfigCheck(
                name=".gitignore",
                status=ConfigCheckStatus.WARNING,
                error_message="No .gitignore file found",
            ))

        # Check for security headers configuration
        # (This would be expanded for specific frameworks)

        return checks


def get_security_scanner() -> SecurityScanner:
    """Get security scanner instance.

    Returns:
        SecurityScanner instance
    """
    return SecurityScanner()
