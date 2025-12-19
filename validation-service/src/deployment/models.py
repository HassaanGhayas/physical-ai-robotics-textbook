"""Data models for deployment validation.

This module defines the data structures for deployment validation results.
"""

from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, Field


class ConfigCheckStatus(str, Enum):
    """Status of a configuration check."""

    VALID = "valid"
    INVALID = "invalid"
    MISSING = "missing"
    WARNING = "warning"


class SeverityLevel(str, Enum):
    """Severity level for issues."""

    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    INFO = "info"


class ConfigCheck(BaseModel):
    """Result of a configuration check."""

    name: str = Field(description="Configuration item name")
    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    expected_type: Optional[str] = Field(default=None, description="Expected type")
    actual_value: Optional[str] = Field(default=None, description="Actual value (redacted if sensitive)")
    is_sensitive: bool = Field(default=False, description="Whether this is a sensitive value")
    error_message: Optional[str] = Field(default=None, description="Error message if invalid")
    location: Optional[str] = Field(default=None, description="File or location of config")


class SecretRisk(BaseModel):
    """Detected secret or credential risk."""

    file_path: str = Field(description="File containing the potential secret")
    line_number: int = Field(description="Line number where detected")
    secret_type: str = Field(description="Type of secret (API key, password, etc.)")
    severity: SeverityLevel = Field(default=SeverityLevel.HIGH)
    pattern_matched: str = Field(description="Pattern that matched")
    recommendation: str = Field(description="Recommended action")


class DependencyConflict(BaseModel):
    """Detected dependency conflict."""

    package: str = Field(description="Package name")
    required_by: list[str] = Field(default_factory=list, description="Packages requiring this")
    versions_required: list[str] = Field(default_factory=list, description="Version requirements")
    conflict_type: str = Field(description="Type of conflict")
    severity: SeverityLevel = Field(default=SeverityLevel.MEDIUM)
    resolution: Optional[str] = Field(default=None, description="Suggested resolution")


class OutdatedPackage(BaseModel):
    """Information about an outdated package."""

    name: str = Field(description="Package name")
    current_version: str = Field(description="Currently installed version")
    latest_version: str = Field(description="Latest available version")
    latest_stable: Optional[str] = Field(default=None, description="Latest stable version")
    severity: SeverityLevel = Field(default=SeverityLevel.LOW)
    has_security_update: bool = Field(default=False, description="Whether update includes security fixes")
    changelog_url: Optional[str] = Field(default=None, description="URL to changelog")


class ResourceCheck(BaseModel):
    """Resource requirement check result."""

    resource_type: str = Field(description="Type of resource (memory, cpu, disk)")
    required: str = Field(description="Required amount")
    available: Optional[str] = Field(default=None, description="Available amount")
    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    message: Optional[str] = Field(default=None)


class PlatformCheck(BaseModel):
    """Platform compatibility check result."""

    platform: str = Field(description="Platform name")
    required_version: Optional[str] = Field(default=None, description="Required version")
    detected_version: Optional[str] = Field(default=None, description="Detected version")
    is_compatible: bool = Field(default=True)
    warnings: list[str] = Field(default_factory=list)


class ServiceCheck(BaseModel):
    """External service dependency check result."""

    service_name: str = Field(description="Service name")
    endpoint: Optional[str] = Field(default=None, description="Service endpoint")
    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    latency_ms: Optional[float] = Field(default=None, description="Response latency")
    error_message: Optional[str] = Field(default=None)
    is_required: bool = Field(default=True, description="Whether service is required")


class ConfigurationValidationResult(BaseModel):
    """Result of configuration validation."""

    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    checks: list[ConfigCheck] = Field(default_factory=list)
    missing_required: list[str] = Field(default_factory=list)
    invalid_values: list[str] = Field(default_factory=list)
    secret_risks: list[SecretRisk] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


class DependencyValidationResult(BaseModel):
    """Result of dependency validation."""

    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    total_dependencies: int = Field(default=0)
    conflicts: list[DependencyConflict] = Field(default_factory=list)
    outdated_packages: list[OutdatedPackage] = Field(default_factory=list)
    security_vulnerabilities: int = Field(default=0)
    warnings: list[str] = Field(default_factory=list)


class CompatibilityValidationResult(BaseModel):
    """Result of compatibility validation."""

    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    resource_checks: list[ResourceCheck] = Field(default_factory=list)
    platform_checks: list[PlatformCheck] = Field(default_factory=list)
    service_checks: list[ServiceCheck] = Field(default_factory=list)
    warnings: list[str] = Field(default_factory=list)


class DeploymentValidationResult(BaseModel):
    """Combined deployment validation result."""

    run_id: UUID = Field(description="Validation run ID")
    validated_at: datetime = Field(default_factory=datetime.utcnow)
    status: ConfigCheckStatus = Field(default=ConfigCheckStatus.VALID)
    configuration: ConfigurationValidationResult = Field(
        default_factory=ConfigurationValidationResult
    )
    dependencies: DependencyValidationResult = Field(
        default_factory=DependencyValidationResult
    )
    compatibility: CompatibilityValidationResult = Field(
        default_factory=CompatibilityValidationResult
    )
    is_deployable: bool = Field(default=True, description="Whether deployment can proceed")
    blocking_issues: list[str] = Field(default_factory=list, description="Issues that block deployment")
    warnings: list[str] = Field(default_factory=list, description="Non-blocking warnings")
