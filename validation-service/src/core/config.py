"""Configuration management for the validation service.

This module provides centralized configuration using pydantic-settings
with environment variable and file-based configuration support.
"""

from functools import lru_cache
from pathlib import Path
from typing import Any

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class QualitySettings(BaseSettings):
    """Code quality validation settings."""

    model_config = SettingsConfigDict(env_prefix="QUALITY_")

    # Linting
    max_line_length: int = Field(default=100, description="Maximum line length for linting")
    enable_security_scan: bool = Field(default=True, description="Enable Bandit security scanning")

    # Complexity thresholds
    max_cyclomatic_complexity: int = Field(
        default=10, description="Maximum cyclomatic complexity per function"
    )
    max_cognitive_complexity: int = Field(
        default=15, description="Maximum cognitive complexity per function"
    )
    min_maintainability_index: float = Field(
        default=20.0, description="Minimum maintainability index (0-100)"
    )

    # Coverage
    min_coverage_percentage: float = Field(
        default=80.0, description="Minimum test coverage percentage"
    )


class TestingSettings(BaseSettings):
    """Test execution settings."""

    model_config = SettingsConfigDict(env_prefix="TESTING_")

    # Execution
    parallel_execution: bool = Field(default=True, description="Enable parallel test execution")
    max_workers: int = Field(default=4, description="Maximum parallel test workers")
    timeout_seconds: int = Field(default=300, description="Test timeout in seconds")

    # Flaky test detection
    flaky_detection_enabled: bool = Field(default=True, description="Enable flaky test detection")
    flaky_min_runs: int = Field(default=10, description="Minimum runs for flaky detection")
    flaky_pass_threshold: float = Field(
        default=0.8, description="Pass rate threshold for flaky detection (0-1)"
    )
    flaky_confidence_level: float = Field(
        default=0.95, description="Statistical confidence level for flaky detection"
    )

    # Retry
    retry_flaky_tests: bool = Field(default=True, description="Automatically retry flaky tests")
    max_retries: int = Field(default=3, description="Maximum retry attempts for flaky tests")


class DeploymentSettings(BaseSettings):
    """Deployment validation settings."""

    model_config = SettingsConfigDict(env_prefix="DEPLOYMENT_")

    # Configuration validation
    validate_env_vars: bool = Field(default=True, description="Validate environment variables")
    validate_secrets: bool = Field(default=True, description="Check for exposed secrets")

    # Dependency checking
    check_vulnerabilities: bool = Field(default=True, description="Check for dependency vulnerabilities")
    check_outdated: bool = Field(default=True, description="Check for outdated dependencies")
    vulnerability_severity_threshold: str = Field(
        default="medium", description="Minimum severity to report (low, medium, high, critical)"
    )

    # Compatibility
    check_resource_requirements: bool = Field(
        default=True, description="Validate resource requirements"
    )
    target_environments: list[str] = Field(
        default_factory=lambda: ["production"], description="Target deployment environments"
    )


class StorageSettings(BaseSettings):
    """Storage and persistence settings."""

    model_config = SettingsConfigDict(env_prefix="STORAGE_")

    # Storage paths
    data_dir: Path = Field(
        default=Path(".validation-data"), description="Base directory for validation data"
    )
    results_dir: Path = Field(
        default=Path(".validation-data/results"), description="Directory for validation results"
    )
    history_dir: Path = Field(
        default=Path(".validation-data/history"), description="Directory for test history"
    )

    # Retention
    retention_days: int = Field(default=30, description="Days to retain validation results")
    max_history_entries: int = Field(
        default=1000, description="Maximum history entries per test"
    )


class APISettings(BaseSettings):
    """API server settings."""

    model_config = SettingsConfigDict(env_prefix="API_")

    host: str = Field(default="0.0.0.0", description="API server host")
    port: int = Field(default=8080, description="API server port")
    debug: bool = Field(default=False, description="Enable debug mode")
    reload: bool = Field(default=False, description="Enable auto-reload (development)")

    # CORS
    cors_origins: list[str] = Field(
        default_factory=lambda: ["*"], description="Allowed CORS origins"
    )

    # Rate limiting
    rate_limit_requests: int = Field(default=100, description="Max requests per minute")


class Settings(BaseSettings):
    """Main application settings.

    Configuration is loaded from:
    1. Environment variables (highest priority)
    2. .env file
    3. Default values (lowest priority)
    """

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        env_nested_delimiter="__",
        extra="ignore",
    )

    # Application metadata
    app_name: str = Field(default="validation-service", description="Application name")
    app_version: str = Field(default="0.1.0", description="Application version")
    environment: str = Field(default="development", description="Runtime environment")
    log_level: str = Field(default="INFO", description="Logging level")

    # Feature flags
    enable_quality_validation: bool = Field(default=True, description="Enable code quality checks")
    enable_test_execution: bool = Field(default=True, description="Enable test suite execution")
    enable_deployment_validation: bool = Field(
        default=True, description="Enable deployment checks"
    )

    # Pipeline settings
    pipeline_timeout_minutes: int = Field(
        default=15, description="Maximum pipeline execution time"
    )
    fail_fast: bool = Field(default=False, description="Stop on first failure")

    # Nested settings
    quality: QualitySettings = Field(default_factory=QualitySettings)
    testing: TestingSettings = Field(default_factory=TestingSettings)
    deployment: DeploymentSettings = Field(default_factory=DeploymentSettings)
    storage: StorageSettings = Field(default_factory=StorageSettings)
    api: APISettings = Field(default_factory=APISettings)

    @field_validator("log_level")
    @classmethod
    def validate_log_level(cls, v: str) -> str:
        """Validate log level is a valid Python logging level."""
        valid_levels = {"DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"}
        upper_v = v.upper()
        if upper_v not in valid_levels:
            raise ValueError(f"Invalid log level: {v}. Must be one of {valid_levels}")
        return upper_v

    @field_validator("environment")
    @classmethod
    def validate_environment(cls, v: str) -> str:
        """Validate environment name."""
        valid_envs = {"development", "staging", "production", "test"}
        lower_v = v.lower()
        if lower_v not in valid_envs:
            raise ValueError(f"Invalid environment: {v}. Must be one of {valid_envs}")
        return lower_v

    def ensure_directories(self) -> None:
        """Create required directories if they don't exist."""
        self.storage.data_dir.mkdir(parents=True, exist_ok=True)
        self.storage.results_dir.mkdir(parents=True, exist_ok=True)
        self.storage.history_dir.mkdir(parents=True, exist_ok=True)

    def to_dict(self) -> dict[str, Any]:
        """Convert settings to dictionary for serialization."""
        return self.model_dump()


@lru_cache
def get_settings() -> Settings:
    """Get cached application settings.

    Returns:
        Settings: Application configuration instance

    Note:
        Settings are cached for performance. Use `get_settings.cache_clear()`
        to force reload if environment changes.
    """
    return Settings()


def get_settings_uncached() -> Settings:
    """Get fresh application settings without caching.

    Returns:
        Settings: Fresh application configuration instance
    """
    return Settings()
