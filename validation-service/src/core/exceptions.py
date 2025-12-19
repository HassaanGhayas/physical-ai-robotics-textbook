"""Exception classes for the validation service.

This module defines custom exceptions for handling validation errors,
pipeline failures, and other error conditions.
"""

from typing import Optional


class ValidationServiceError(Exception):
    """Base exception for all validation service errors."""

    def __init__(self, message: str, code: Optional[str] = None):
        """Initialize exception.

        Args:
            message: Error message
            code: Optional error code for categorization
        """
        super().__init__(message)
        self.message = message
        self.code = code or "UNKNOWN_ERROR"

    def __str__(self) -> str:
        return f"[{self.code}] {self.message}"


# ============================================================================
# Configuration Errors
# ============================================================================


class ConfigurationError(ValidationServiceError):
    """Error in service configuration."""

    def __init__(self, message: str):
        super().__init__(message, "CONFIG_ERROR")


class MissingConfigError(ConfigurationError):
    """Required configuration is missing."""

    def __init__(self, key: str):
        super().__init__(f"Missing required configuration: {key}")
        self.key = key


class InvalidConfigError(ConfigurationError):
    """Configuration value is invalid."""

    def __init__(self, key: str, value: str, reason: str):
        super().__init__(f"Invalid configuration '{key}={value}': {reason}")
        self.key = key
        self.value = value
        self.reason = reason


# ============================================================================
# Pipeline Errors
# ============================================================================


class PipelineError(ValidationServiceError):
    """Error during pipeline execution."""

    def __init__(self, message: str, stage: Optional[str] = None):
        super().__init__(message, "PIPELINE_ERROR")
        self.stage = stage


class PipelineTimeoutError(PipelineError):
    """Pipeline execution exceeded timeout."""

    def __init__(self, timeout_seconds: float):
        super().__init__(f"Pipeline timed out after {timeout_seconds:.1f}s")
        self.timeout_seconds = timeout_seconds


class StageExecutionError(PipelineError):
    """Error executing a specific pipeline stage."""

    def __init__(self, stage: str, message: str, cause: Optional[Exception] = None):
        super().__init__(f"Stage '{stage}' failed: {message}", stage)
        self.cause = cause


class StageDependencyError(PipelineError):
    """Required stage dependency not met."""

    def __init__(self, stage: str, required_stage: str):
        super().__init__(
            f"Stage '{stage}' requires '{required_stage}' to complete first",
            stage,
        )
        self.required_stage = required_stage


# ============================================================================
# Quality Validation Errors
# ============================================================================


class QualityValidationError(ValidationServiceError):
    """Error during code quality validation."""

    def __init__(self, message: str):
        super().__init__(message, "QUALITY_ERROR")


class LinterError(QualityValidationError):
    """Error running linter."""

    def __init__(self, linter: str, message: str):
        super().__init__(f"Linter '{linter}' failed: {message}")
        self.linter = linter


class ComplexityAnalysisError(QualityValidationError):
    """Error during complexity analysis."""

    def __init__(self, message: str):
        super().__init__(f"Complexity analysis failed: {message}")


class CoverageError(QualityValidationError):
    """Error calculating test coverage."""

    def __init__(self, message: str):
        super().__init__(f"Coverage calculation failed: {message}")


class SecurityScanError(QualityValidationError):
    """Error during security scanning."""

    def __init__(self, scanner: str, message: str):
        super().__init__(f"Security scanner '{scanner}' failed: {message}")
        self.scanner = scanner


# ============================================================================
# Test Execution Errors
# ============================================================================


class TestExecutionError(ValidationServiceError):
    """Error during test execution."""

    def __init__(self, message: str, test_type: Optional[str] = None):
        super().__init__(message, "TEST_ERROR")
        self.test_type = test_type


class TestRunnerError(TestExecutionError):
    """Error with test runner."""

    def __init__(self, runner: str, message: str):
        super().__init__(f"Test runner '{runner}' failed: {message}")
        self.runner = runner


class TestFrameworkNotFoundError(TestExecutionError):
    """Test framework not found or not installed."""

    def __init__(self, framework: str):
        super().__init__(f"Test framework '{framework}' not found")
        self.framework = framework


class TestTimeoutError(TestExecutionError):
    """Test execution exceeded timeout."""

    def __init__(self, test_id: str, timeout_seconds: float):
        super().__init__(f"Test '{test_id}' timed out after {timeout_seconds:.1f}s")
        self.test_id = test_id
        self.timeout_seconds = timeout_seconds


class NoTestsFoundError(TestExecutionError):
    """No tests found in project."""

    def __init__(self, test_type: str, search_paths: list[str]):
        super().__init__(
            f"No {test_type} tests found in: {', '.join(search_paths)}",
            test_type,
        )
        self.search_paths = search_paths


# ============================================================================
# Deployment Validation Errors
# ============================================================================


class DeploymentValidationError(ValidationServiceError):
    """Error during deployment validation."""

    def __init__(self, message: str):
        super().__init__(message, "DEPLOYMENT_ERROR")


class ConfigValidationError(DeploymentValidationError):
    """Configuration validation failed."""

    def __init__(self, issues: list[str]):
        super().__init__(f"Configuration validation failed: {len(issues)} issues found")
        self.issues = issues


class DependencyCheckError(DeploymentValidationError):
    """Dependency check failed."""

    def __init__(self, message: str, package: Optional[str] = None):
        super().__init__(f"Dependency check failed: {message}")
        self.package = package


class VulnerabilityFoundError(DeploymentValidationError):
    """Security vulnerability found in dependencies."""

    def __init__(self, package: str, severity: str, cve_id: Optional[str] = None):
        cve_str = f" ({cve_id})" if cve_id else ""
        super().__init__(
            f"Vulnerability found in '{package}': {severity} severity{cve_str}"
        )
        self.package = package
        self.severity = severity
        self.cve_id = cve_id


class CompatibilityError(DeploymentValidationError):
    """Environment compatibility check failed."""

    def __init__(self, check_type: str, required: str, actual: str):
        super().__init__(
            f"Compatibility check failed: {check_type} requires '{required}', got '{actual}'"
        )
        self.check_type = check_type
        self.required = required
        self.actual = actual


# ============================================================================
# Storage Errors
# ============================================================================


class StorageError(ValidationServiceError):
    """Error with data storage."""

    def __init__(self, message: str):
        super().__init__(message, "STORAGE_ERROR")


class StorageReadError(StorageError):
    """Error reading from storage."""

    def __init__(self, path: str, reason: str):
        super().__init__(f"Failed to read '{path}': {reason}")
        self.path = path
        self.reason = reason


class StorageWriteError(StorageError):
    """Error writing to storage."""

    def __init__(self, path: str, reason: str):
        super().__init__(f"Failed to write '{path}': {reason}")
        self.path = path
        self.reason = reason


class ValidationNotFoundError(StorageError):
    """Validation run not found."""

    def __init__(self, run_id: str):
        super().__init__(f"Validation run '{run_id}' not found")
        self.run_id = run_id


class TestCaseNotFoundError(StorageError):
    """Test case not found."""

    def __init__(self, test_id: str):
        super().__init__(f"Test case '{test_id}' not found")
        self.test_id = test_id


# ============================================================================
# API Errors
# ============================================================================


class APIError(ValidationServiceError):
    """Error in API layer."""

    def __init__(self, message: str, status_code: int = 500):
        super().__init__(message, "API_ERROR")
        self.status_code = status_code


class BadRequestError(APIError):
    """Invalid request error (400)."""

    def __init__(self, message: str):
        super().__init__(message, 400)


class NotFoundError(APIError):
    """Resource not found error (404)."""

    def __init__(self, resource: str, identifier: str):
        super().__init__(f"{resource} '{identifier}' not found", 404)
        self.resource = resource
        self.identifier = identifier


class ConflictError(APIError):
    """Resource conflict error (409)."""

    def __init__(self, message: str):
        super().__init__(message, 409)


class RateLimitError(APIError):
    """Rate limit exceeded error (429)."""

    def __init__(self, retry_after: int):
        super().__init__(f"Rate limit exceeded. Retry after {retry_after}s", 429)
        self.retry_after = retry_after
