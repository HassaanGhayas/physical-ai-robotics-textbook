"""Configuration schema and validation for the validation service."""

from typing import Any, Optional
from pathlib import Path
import json

from pydantic import BaseModel, Field, field_validator, model_validator


class QualityConfig(BaseModel):
    """Configuration for quality validation."""

    enabled: bool = Field(default=True, description="Enable quality validation")
    min_coverage_percentage: float = Field(
        default=80.0, ge=0, le=100, description="Minimum code coverage percentage"
    )
    max_cyclomatic_complexity: int = Field(
        default=10, ge=1, description="Maximum cyclomatic complexity"
    )
    max_cognitive_complexity: int = Field(
        default=15, ge=1, description="Maximum cognitive complexity"
    )
    max_line_length: int = Field(
        default=120, ge=40, le=500, description="Maximum line length"
    )
    max_function_length: int = Field(
        default=50, ge=10, description="Maximum function length in lines"
    )
    enable_security_scan: bool = Field(
        default=True, description="Enable security scanning"
    )
    enable_linting: bool = Field(default=True, description="Enable linting checks")
    enable_type_checking: bool = Field(
        default=True, description="Enable type checking"
    )
    excluded_paths: list[str] = Field(
        default_factory=lambda: ["tests/", "docs/", ".venv/"],
        description="Paths to exclude from quality checks",
    )


class TestingConfig(BaseModel):
    """Configuration for test execution."""

    enabled: bool = Field(default=True, description="Enable test execution")
    parallel_execution: bool = Field(
        default=True, description="Run tests in parallel"
    )
    max_workers: int = Field(
        default=4, ge=1, le=32, description="Maximum parallel workers"
    )
    timeout_seconds: int = Field(
        default=300, ge=60, le=3600, description="Test timeout in seconds"
    )
    flaky_detection_enabled: bool = Field(
        default=True, description="Enable flaky test detection"
    )
    flaky_min_runs: int = Field(
        default=10, ge=5, description="Minimum runs for flaky detection"
    )
    flaky_pass_threshold: float = Field(
        default=0.8, ge=0.5, le=1.0, description="Pass rate threshold for flaky detection"
    )
    flaky_confidence_level: float = Field(
        default=0.95, ge=0.8, le=0.99, description="Confidence level for flaky detection"
    )
    retry_flaky_tests: bool = Field(
        default=True, description="Retry detected flaky tests"
    )
    max_retries: int = Field(default=3, ge=1, le=10, description="Maximum retries")
    run_unit_tests: bool = Field(default=True, description="Run unit tests")
    run_integration_tests: bool = Field(
        default=True, description="Run integration tests"
    )
    run_e2e_tests: bool = Field(default=True, description="Run E2E tests")
    test_paths: dict[str, str] = Field(
        default_factory=lambda: {
            "unit": "tests/unit/",
            "integration": "tests/integration/",
            "e2e": "tests/e2e/",
        },
        description="Test directory paths",
    )


class DeploymentConfig(BaseModel):
    """Configuration for deployment validation."""

    enabled: bool = Field(default=True, description="Enable deployment validation")
    require_passing_tests: bool = Field(
        default=True, description="Require all tests to pass"
    )
    require_passing_quality: bool = Field(
        default=True, description="Require quality checks to pass"
    )
    check_vulnerabilities: bool = Field(
        default=True, description="Check for security vulnerabilities"
    )
    check_outdated_dependencies: bool = Field(
        default=True, description="Check for outdated dependencies"
    )
    vulnerability_severity_threshold: str = Field(
        default="medium",
        description="Minimum severity level to fail (low, medium, high, critical)",
    )
    check_environment_variables: bool = Field(
        default=True, description="Validate required environment variables"
    )
    required_env_vars: list[str] = Field(
        default_factory=list, description="Required environment variables"
    )
    allowed_environments: list[str] = Field(
        default_factory=lambda: ["development", "staging", "production"],
        description="Allowed deployment environments",
    )

    @field_validator("vulnerability_severity_threshold")
    @classmethod
    def validate_severity(cls, v: str) -> str:
        """Validate severity threshold."""
        valid = ["low", "medium", "high", "critical"]
        if v.lower() not in valid:
            raise ValueError(f"severity must be one of {valid}")
        return v.lower()


class PipelineConfig(BaseModel):
    """Configuration for the validation pipeline."""

    timeout_minutes: int = Field(
        default=15, ge=1, le=120, description="Pipeline timeout in minutes"
    )
    fail_fast: bool = Field(
        default=False, description="Stop on first failure"
    )
    notify_on_failure: bool = Field(
        default=True, description="Send notifications on failure"
    )
    notify_on_success: bool = Field(
        default=False, description="Send notifications on success"
    )
    save_artifacts: bool = Field(
        default=True, description="Save validation artifacts"
    )
    artifacts_path: str = Field(
        default=".validation-data/artifacts",
        description="Path for validation artifacts",
    )


class StorageConfig(BaseModel):
    """Configuration for data storage."""

    data_dir: str = Field(
        default=".validation-data", description="Base data directory"
    )
    results_dir: str = Field(
        default=".validation-data/results", description="Results directory"
    )
    history_dir: str = Field(
        default=".validation-data/history", description="History directory"
    )
    retention_days: int = Field(
        default=30, ge=1, le=365, description="Data retention in days"
    )
    max_history_entries: int = Field(
        default=1000, ge=100, le=10000, description="Maximum history entries"
    )


class APIConfig(BaseModel):
    """Configuration for the API server."""

    host: str = Field(default="0.0.0.0", description="API host")
    port: int = Field(default=8000, ge=1, le=65535, description="API port")
    debug: bool = Field(default=False, description="Enable debug mode")
    reload: bool = Field(default=False, description="Enable auto-reload")
    cors_origins: list[str] = Field(
        default_factory=lambda: ["*"], description="Allowed CORS origins"
    )
    rate_limit_requests: int = Field(
        default=100, ge=10, description="Rate limit requests per minute"
    )


class ValidationServiceConfig(BaseModel):
    """Root configuration for the validation service."""

    environment: str = Field(
        default="development",
        description="Environment (development, staging, production)",
    )
    log_level: str = Field(
        default="INFO", description="Logging level"
    )
    log_format: str = Field(
        default="text", description="Log format (text, json)"
    )

    quality: QualityConfig = Field(default_factory=QualityConfig)
    testing: TestingConfig = Field(default_factory=TestingConfig)
    deployment: DeploymentConfig = Field(default_factory=DeploymentConfig)
    pipeline: PipelineConfig = Field(default_factory=PipelineConfig)
    storage: StorageConfig = Field(default_factory=StorageConfig)
    api: APIConfig = Field(default_factory=APIConfig)

    @field_validator("environment")
    @classmethod
    def validate_environment(cls, v: str) -> str:
        """Validate environment."""
        valid = ["development", "staging", "production", "test", "ci"]
        if v.lower() not in valid:
            raise ValueError(f"environment must be one of {valid}")
        return v.lower()

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level."""
        valid = ["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"]
        if v.upper() not in valid:
            raise ValueError(f"log_level must be one of {valid}")
        return v.upper()

    @field_validator("log_format")
    @classmethod
    def validate_log_format(cls, v: str) -> str:
        """Validate log format."""
        valid = ["text", "json"]
        if v.lower() not in valid:
            raise ValueError(f"log_format must be one of {valid}")
        return v.lower()

    @model_validator(mode="after")
    def validate_config(self) -> "ValidationServiceConfig":
        """Validate configuration consistency."""
        # Ensure deployment requirements match enabled features
        if self.deployment.require_passing_tests and not self.testing.enabled:
            raise ValueError(
                "Cannot require passing tests when testing is disabled"
            )
        if self.deployment.require_passing_quality and not self.quality.enabled:
            raise ValueError(
                "Cannot require passing quality when quality validation is disabled"
            )
        return self

    def to_dict(self) -> dict[str, Any]:
        """Convert configuration to dictionary."""
        return self.model_dump()

    def to_json(self, indent: int = 2) -> str:
        """Convert configuration to JSON string."""
        return self.model_dump_json(indent=indent)

    @classmethod
    def from_file(cls, path: Path) -> "ValidationServiceConfig":
        """Load configuration from file.

        Args:
            path: Path to configuration file (JSON or YAML)

        Returns:
            Loaded configuration
        """
        content = path.read_text()

        if path.suffix in [".yaml", ".yml"]:
            import yaml

            data = yaml.safe_load(content)
        else:
            data = json.loads(content)

        return cls(**data)

    def save(self, path: Path) -> None:
        """Save configuration to file.

        Args:
            path: Path to save configuration
        """
        if path.suffix in [".yaml", ".yml"]:
            import yaml

            content = yaml.dump(self.to_dict(), default_flow_style=False)
        else:
            content = self.to_json()

        path.write_text(content)


def get_json_schema() -> dict[str, Any]:
    """Get JSON schema for configuration.

    Returns:
        JSON schema dictionary
    """
    return ValidationServiceConfig.model_json_schema()


def validate_config(config: dict[str, Any]) -> list[str]:
    """Validate configuration dictionary.

    Args:
        config: Configuration dictionary

    Returns:
        List of validation errors (empty if valid)
    """
    errors = []
    try:
        ValidationServiceConfig(**config)
    except Exception as e:
        errors.append(str(e))
    return errors
