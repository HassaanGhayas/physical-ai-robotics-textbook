"""Configuration validator service.

This module provides configuration validation using JSON Schema
for deployment configuration files.
"""

import json
import logging
import os
import re
from pathlib import Path
from typing import Any, Optional

from src.core.config import get_settings
from src.deployment.models import (
    ConfigCheck,
    ConfigCheckStatus,
    SecretRisk,
    SeverityLevel,
    ConfigurationValidationResult,
)


logger = logging.getLogger(__name__)


# Common patterns for detecting secrets
SECRET_PATTERNS = [
    (r"(?i)(api[_-]?key|apikey)\s*[=:]\s*['\"][^'\"]{8,}['\"]", "API Key"),
    (r"(?i)(secret|password|passwd|pwd)\s*[=:]\s*['\"][^'\"]{4,}['\"]", "Password/Secret"),
    (r"(?i)(token|access[_-]?token|auth[_-]?token)\s*[=:]\s*['\"][^'\"]{10,}['\"]", "Token"),
    (r"(?i)(aws[_-]?access[_-]?key|aws[_-]?secret)", "AWS Credentials"),
    (r"(?i)(private[_-]?key|ssh[_-]?key)", "Private Key"),
    (r"(?i)(database[_-]?url|db[_-]?password|connection[_-]?string)", "Database Credentials"),
    (r"-----BEGIN\s+(RSA\s+)?PRIVATE\s+KEY-----", "Private Key (PEM)"),
    (r"ghp_[a-zA-Z0-9]{36}", "GitHub Personal Access Token"),
    (r"sk-[a-zA-Z0-9]{48}", "OpenAI API Key"),
]


class ConfigValidator:
    """Service for validating deployment configuration."""

    # Common environment variable requirements
    COMMON_REQUIRED_VARS = [
        "NODE_ENV",
        "LOG_LEVEL",
    ]

    # Sensitive variable patterns
    SENSITIVE_PATTERNS = [
        r".*password.*",
        r".*secret.*",
        r".*key.*",
        r".*token.*",
        r".*credential.*",
    ]

    def __init__(self):
        """Initialize the configuration validator."""
        self.settings = get_settings()

    def validate(
        self,
        project_path: Path,
        schema_path: Optional[Path] = None,
        env_file: str = ".env",
    ) -> ConfigurationValidationResult:
        """Validate project configuration.

        Args:
            project_path: Path to project root
            schema_path: Optional path to JSON schema for validation
            env_file: Name of environment file to validate

        Returns:
            ConfigurationValidationResult
        """
        result = ConfigurationValidationResult()

        # Check for environment file
        env_path = project_path / env_file
        if env_path.exists():
            env_checks = self._validate_env_file(env_path)
            result.checks.extend(env_checks)

        # Check for example environment file
        env_example_path = project_path / f"{env_file}.example"
        if env_example_path.exists():
            missing = self._check_env_completeness(env_path, env_example_path)
            result.missing_required.extend(missing)

        # Validate JSON/YAML config files
        config_files = self._find_config_files(project_path)
        for config_file in config_files:
            file_checks = self._validate_config_file(config_file, schema_path)
            result.checks.extend(file_checks)

        # Scan for hardcoded secrets
        secrets = self._scan_for_secrets(project_path)
        result.secret_risks.extend(secrets)

        # Determine overall status
        has_missing = len(result.missing_required) > 0
        has_invalid = any(c.status == ConfigCheckStatus.INVALID for c in result.checks)
        has_secrets = len(result.secret_risks) > 0

        if has_invalid or has_secrets:
            result.status = ConfigCheckStatus.INVALID
        elif has_missing:
            result.status = ConfigCheckStatus.WARNING
        else:
            result.status = ConfigCheckStatus.VALID

        return result

    def _validate_env_file(self, env_path: Path) -> list[ConfigCheck]:
        """Validate environment file.

        Args:
            env_path: Path to .env file

        Returns:
            List of ConfigCheck results
        """
        checks = []

        try:
            content = env_path.read_text()
            lines = content.split("\n")

            for line_num, line in enumerate(lines, 1):
                line = line.strip()

                # Skip comments and empty lines
                if not line or line.startswith("#"):
                    continue

                if "=" not in line:
                    checks.append(ConfigCheck(
                        name=f"Line {line_num}",
                        status=ConfigCheckStatus.INVALID,
                        error_message="Invalid format: missing '=' separator",
                        location=str(env_path),
                    ))
                    continue

                key, value = line.split("=", 1)
                key = key.strip()
                value = value.strip().strip('"').strip("'")

                # Check if sensitive
                is_sensitive = any(
                    re.match(pattern, key, re.IGNORECASE)
                    for pattern in self.SENSITIVE_PATTERNS
                )

                # Check for empty required values
                if not value and key in self.COMMON_REQUIRED_VARS:
                    checks.append(ConfigCheck(
                        name=key,
                        status=ConfigCheckStatus.MISSING,
                        is_sensitive=is_sensitive,
                        error_message="Required variable has empty value",
                        location=str(env_path),
                    ))
                else:
                    checks.append(ConfigCheck(
                        name=key,
                        status=ConfigCheckStatus.VALID,
                        actual_value="***" if is_sensitive else value[:50],
                        is_sensitive=is_sensitive,
                        location=str(env_path),
                    ))

        except Exception as e:
            logger.error(f"Error reading env file: {e}")
            checks.append(ConfigCheck(
                name=str(env_path),
                status=ConfigCheckStatus.INVALID,
                error_message=f"Failed to read file: {e}",
            ))

        return checks

    def _check_env_completeness(
        self,
        env_path: Path,
        example_path: Path,
    ) -> list[str]:
        """Check if .env has all variables from .env.example.

        Args:
            env_path: Path to actual .env file
            example_path: Path to .env.example file

        Returns:
            List of missing variable names
        """
        missing = []

        try:
            example_vars = self._parse_env_vars(example_path)
            actual_vars = self._parse_env_vars(env_path) if env_path.exists() else set()

            for var in example_vars:
                if var not in actual_vars:
                    missing.append(var)

        except Exception as e:
            logger.error(f"Error checking env completeness: {e}")

        return missing

    def _parse_env_vars(self, env_path: Path) -> set[str]:
        """Parse variable names from an env file.

        Args:
            env_path: Path to env file

        Returns:
            Set of variable names
        """
        vars_found = set()

        if not env_path.exists():
            return vars_found

        content = env_path.read_text()
        for line in content.split("\n"):
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key = line.split("=", 1)[0].strip()
                vars_found.add(key)

        return vars_found

    def _find_config_files(self, project_path: Path) -> list[Path]:
        """Find configuration files in the project.

        Args:
            project_path: Path to project root

        Returns:
            List of config file paths
        """
        config_files = []
        patterns = [
            "config.json",
            "config.yaml",
            "config.yml",
            "settings.json",
            "settings.yaml",
            ".deployment/*.json",
            ".deployment/*.yaml",
        ]

        for pattern in patterns:
            config_files.extend(project_path.glob(pattern))

        return config_files

    def _validate_config_file(
        self,
        config_path: Path,
        schema_path: Optional[Path],
    ) -> list[ConfigCheck]:
        """Validate a configuration file.

        Args:
            config_path: Path to config file
            schema_path: Optional JSON schema path

        Returns:
            List of ConfigCheck results
        """
        checks = []

        try:
            # Validate JSON syntax
            if config_path.suffix == ".json":
                with open(config_path) as f:
                    data = json.load(f)
                checks.append(ConfigCheck(
                    name=str(config_path.name),
                    status=ConfigCheckStatus.VALID,
                    location=str(config_path),
                ))

                # Validate against schema if provided
                if schema_path and schema_path.exists():
                    schema_checks = self._validate_against_schema(data, schema_path)
                    checks.extend(schema_checks)

        except json.JSONDecodeError as e:
            checks.append(ConfigCheck(
                name=str(config_path.name),
                status=ConfigCheckStatus.INVALID,
                error_message=f"Invalid JSON: {e}",
                location=str(config_path),
            ))
        except Exception as e:
            checks.append(ConfigCheck(
                name=str(config_path.name),
                status=ConfigCheckStatus.INVALID,
                error_message=f"Error reading file: {e}",
                location=str(config_path),
            ))

        return checks

    def _validate_against_schema(
        self,
        data: Any,
        schema_path: Path,
    ) -> list[ConfigCheck]:
        """Validate data against JSON schema.

        Args:
            data: Data to validate
            schema_path: Path to JSON schema

        Returns:
            List of ConfigCheck results
        """
        checks = []

        try:
            import jsonschema

            with open(schema_path) as f:
                schema = json.load(f)

            jsonschema.validate(instance=data, schema=schema)
            checks.append(ConfigCheck(
                name="Schema Validation",
                status=ConfigCheckStatus.VALID,
            ))

        except ImportError:
            logger.warning("jsonschema not installed, skipping schema validation")
        except jsonschema.ValidationError as e:
            checks.append(ConfigCheck(
                name="Schema Validation",
                status=ConfigCheckStatus.INVALID,
                error_message=str(e.message),
            ))
        except Exception as e:
            logger.error(f"Schema validation error: {e}")

        return checks

    def _scan_for_secrets(self, project_path: Path) -> list[SecretRisk]:
        """Scan project for hardcoded secrets.

        Args:
            project_path: Path to project root

        Returns:
            List of SecretRisk findings
        """
        secrets = []

        # Files to scan
        patterns = ["*.py", "*.js", "*.ts", "*.json", "*.yaml", "*.yml", "*.env*"]

        # Directories to skip
        skip_dirs = {".git", "node_modules", "__pycache__", "venv", ".venv", "dist", "build"}

        for pattern in patterns:
            for file_path in project_path.rglob(pattern):
                # Skip excluded directories
                if any(skip_dir in file_path.parts for skip_dir in skip_dirs):
                    continue

                file_secrets = self._scan_file_for_secrets(file_path)
                secrets.extend(file_secrets)

        return secrets

    def _scan_file_for_secrets(self, file_path: Path) -> list[SecretRisk]:
        """Scan a single file for secrets.

        Args:
            file_path: Path to file

        Returns:
            List of SecretRisk findings
        """
        secrets = []

        try:
            content = file_path.read_text(errors="ignore")
            lines = content.split("\n")

            for line_num, line in enumerate(lines, 1):
                for pattern, secret_type in SECRET_PATTERNS:
                    if re.search(pattern, line):
                        secrets.append(SecretRisk(
                            file_path=str(file_path),
                            line_number=line_num,
                            secret_type=secret_type,
                            severity=SeverityLevel.HIGH,
                            pattern_matched=pattern[:30] + "...",
                            recommendation=f"Move {secret_type} to environment variable",
                        ))

        except Exception as e:
            logger.debug(f"Error scanning file {file_path}: {e}")

        return secrets


def get_config_validator() -> ConfigValidator:
    """Get configuration validator instance.

    Returns:
        ConfigValidator instance
    """
    return ConfigValidator()
