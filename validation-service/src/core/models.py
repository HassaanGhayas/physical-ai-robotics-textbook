"""Core data models for the validation service.

This module defines all Pydantic models based on the data model specification.
Models are designed for JSON serialization and API communication.
"""

from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field, field_validator, model_validator


# ============================================================================
# Enums
# ============================================================================


class TriggerSource(str, Enum):
    """How a validation was triggered."""

    MANUAL = "manual"
    PRE_COMMIT = "pre_commit"
    CI_CD = "ci_cd"
    SCHEDULED = "scheduled"


class ValidationStatus(str, Enum):
    """Status of a validation run."""

    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    TIMEOUT = "timeout"


class CheckStatus(str, Enum):
    """Status of individual checks."""

    PASSED = "passed"
    FAILED = "failed"
    WARNING = "warning"


class Severity(str, Enum):
    """Severity levels for violations and vulnerabilities."""

    CRITICAL = "critical"
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"
    INFO = "info"
    ERROR = "error"
    WARNING = "warning"


class TestSuiteType(str, Enum):
    """Types of test suites."""

    UNIT = "unit"
    INTEGRATION = "integration"
    E2E = "e2e"


class TestStatus(str, Enum):
    """Status of a test execution."""

    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    SKIPPED = "skipped"


class FlakyStatus(str, Enum):
    """Status of a flaky test record."""

    ACTIVE_FLAKY = "active_flaky"
    RESOLVED = "resolved"
    FALSE_POSITIVE = "false_positive"


class SecretType(str, Enum):
    """Types of secrets that can be detected."""

    API_KEY = "api_key"
    PASSWORD = "password"
    TOKEN = "token"
    CERTIFICATE = "certificate"


class ResourceType(str, Enum):
    """Types of resources to check."""

    CPU = "cpu"
    MEMORY = "memory"
    DISK = "disk"
    NETWORK = "network"


class PlatformCheckType(str, Enum):
    """Types of platform compatibility checks."""

    OS = "os"
    RUNTIME = "runtime"
    ARCHITECTURE = "architecture"


# ============================================================================
# Code Quality Models
# ============================================================================


class Violation(BaseModel):
    """A code style violation detected by linting."""

    file: str = Field(..., description="File path relative to repository root")
    line: int = Field(..., ge=1, description="Line number where violation occurs")
    column: Optional[int] = Field(None, ge=1, description="Column number")
    rule: str = Field(..., description="Linting rule identifier (e.g., E501)")
    severity: Severity = Field(..., description="Violation severity")
    message: str = Field(..., description="Human-readable violation description")


class Vulnerability(BaseModel):
    """A security vulnerability found in dependencies."""

    package: str = Field(..., description="Affected package name")
    version: str = Field(..., description="Vulnerable version")
    severity: Severity = Field(..., description="Vulnerability severity")
    cve_id: Optional[str] = Field(None, description="CVE identifier")
    description: str = Field(..., description="Vulnerability description")
    fixed_version: Optional[str] = Field(None, description="Version that fixes the issue")


class LintingResults(BaseModel):
    """Results from linting analysis."""

    violations: list[Violation] = Field(default_factory=list, description="Style issues found")
    total_violations: int = Field(default=0, description="Count of all violations")
    error_count: int = Field(default=0, description="Critical violations")
    warning_count: int = Field(default=0, description="Non-critical violations")


class ComplexityResults(BaseModel):
    """Results from complexity analysis."""

    average_complexity: float = Field(default=0.0, ge=0, description="Mean cyclomatic complexity")
    max_complexity: int = Field(default=0, ge=0, description="Highest complexity in codebase")
    high_complexity_functions: list[str] = Field(
        default_factory=list, description="Functions above threshold (>10)"
    )


class CoverageByType(BaseModel):
    """Coverage breakdown by test type."""

    unit: float = Field(default=0.0, ge=0, le=100, description="Unit test coverage")
    integration: float = Field(default=0.0, ge=0, le=100, description="Integration test coverage")
    e2e: float = Field(default=0.0, ge=0, le=100, description="E2E test coverage")


class CoverageResults(BaseModel):
    """Test coverage metrics."""

    line_coverage_percent: float = Field(default=0.0, ge=0, le=100, description="Line coverage")
    branch_coverage_percent: float = Field(
        default=0.0, ge=0, le=100, description="Branch coverage"
    )
    uncovered_lines: int = Field(default=0, ge=0, description="Count of uncovered lines")
    by_test_type: CoverageByType = Field(
        default_factory=CoverageByType, description="Coverage breakdown"
    )


class SecurityResults(BaseModel):
    """Security scan results."""

    vulnerabilities: list[Vulnerability] = Field(
        default_factory=list, description="Security issues found"
    )
    critical_count: int = Field(default=0, ge=0, description="Critical security issues")
    high_count: int = Field(default=0, ge=0, description="High severity issues")
    medium_count: int = Field(default=0, ge=0, description="Medium severity issues")
    low_count: int = Field(default=0, ge=0, description="Low severity issues")


class QualitySummary(BaseModel):
    """Overall quality summary."""

    maintainability_index: float = Field(
        default=0.0, ge=0, le=100, description="Code maintainability score"
    )
    quality_gate_passed: bool = Field(default=False, description="Whether quality standards met")


class CodeQualityReport(BaseModel):
    """Aggregated results from static code analysis."""

    report_id: UUID = Field(default_factory=uuid4, description="Unique identifier")
    run_id: UUID = Field(..., description="Foreign key to ValidationRun")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When analysis completed")
    status: CheckStatus = Field(default=CheckStatus.PASSED, description="Overall status")
    linting: LintingResults = Field(default_factory=LintingResults, description="Linting results")
    complexity: ComplexityResults = Field(
        default_factory=ComplexityResults, description="Complexity analysis"
    )
    coverage: CoverageResults = Field(default_factory=CoverageResults, description="Coverage metrics")
    security: SecurityResults = Field(
        default_factory=SecurityResults, description="Security scan results"
    )
    summary: QualitySummary = Field(default_factory=QualitySummary, description="Quality summary")


# ============================================================================
# Test Suite Models
# ============================================================================


class TestCaseResult(BaseModel):
    """Result of a single test case execution."""

    test_id: str = Field(..., description="Unique test identifier (fully qualified name)")
    test_name: str = Field(..., description="Human-readable test name")
    status: TestStatus = Field(..., description="Test result status")
    duration_seconds: float = Field(..., ge=0, description="Individual test execution time")
    error_message: Optional[str] = Field(None, description="Error message if failed")
    stack_trace: Optional[str] = Field(None, description="Full stack trace if failed")
    assertions: int = Field(default=0, ge=0, description="Number of assertions in test")
    retries: int = Field(default=0, ge=0, description="Number of retries (flaky test detection)")


class TestSuiteResult(BaseModel):
    """Results from executing a test suite."""

    result_id: UUID = Field(default_factory=uuid4, description="Unique identifier")
    run_id: UUID = Field(..., description="Foreign key to ValidationRun")
    suite_type: TestSuiteType = Field(..., description="Type of test suite")
    status: TestStatus = Field(default=TestStatus.PASSED, description="Overall suite status")
    started_at: datetime = Field(default_factory=datetime.utcnow, description="Suite start time")
    completed_at: Optional[datetime] = Field(None, description="Suite completion time")
    duration_seconds: float = Field(default=0.0, ge=0, description="Suite execution time")
    total_tests: int = Field(default=0, ge=0, description="Total number of tests")
    passed_count: int = Field(default=0, ge=0, description="Passing tests")
    failed_count: int = Field(default=0, ge=0, description="Failing tests")
    skipped_count: int = Field(default=0, ge=0, description="Skipped tests")
    error_count: int = Field(default=0, ge=0, description="Tests with errors")
    test_cases: list[TestCaseResult] = Field(
        default_factory=list, description="Individual test results"
    )
    flaky_tests_detected: list[str] = Field(
        default_factory=list, description="Test IDs identified as flaky"
    )
    coverage_contribution: float = Field(
        default=0.0, ge=0, le=100, description="Coverage added by suite"
    )

    @model_validator(mode="after")
    def validate_test_counts(self) -> "TestSuiteResult":
        """Validate that test counts add up to total."""
        calculated = self.passed_count + self.failed_count + self.skipped_count + self.error_count
        if calculated != self.total_tests and self.total_tests != 0:
            raise ValueError(
                f"Test counts ({calculated}) don't match total_tests ({self.total_tests})"
            )
        return self


# ============================================================================
# Test Case & Flaky Test Models
# ============================================================================


class ExecutionRecord(BaseModel):
    """A single test execution record for history tracking."""

    run_id: UUID = Field(..., description="Reference to ValidationRun")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="When test executed")
    status: TestStatus = Field(..., description="Test result status")
    duration_seconds: float = Field(..., ge=0, description="Execution time")
    commit_sha: str = Field(..., description="Commit where test executed")


class TestCase(BaseModel):
    """A test case with execution history for flaky detection."""

    test_id: str = Field(..., description="Unique test identifier (fully qualified name)")
    test_name: str = Field(..., description="Human-readable test name")
    test_type: TestSuiteType = Field(..., description="Test type (unit/integration/e2e)")
    file_path: str = Field(..., description="Test file location")
    target_functionality: str = Field(..., description="What feature/module the test validates")
    execution_history: list[ExecutionRecord] = Field(
        default_factory=list, description="Last N executions (max 10)"
    )
    flakiness_score: float = Field(
        default=0.0, ge=0, le=1, description="Probability of flakiness (1 - pass_rate)"
    )
    is_flaky: bool = Field(default=False, description="Whether currently identified as flaky")
    average_duration_seconds: float = Field(
        default=0.0, ge=0, description="Mean execution time over history"
    )
    last_execution: Optional[datetime] = Field(None, description="Timestamp of most recent execution")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When test was first recorded")
    last_modified: Optional[datetime] = Field(None, description="When test code was last changed")

    @field_validator("execution_history")
    @classmethod
    def limit_history(cls, v: list[ExecutionRecord]) -> list[ExecutionRecord]:
        """Limit execution history to last 10 runs."""
        return v[-10:] if len(v) > 10 else v


class FailureMode(BaseModel):
    """A distinct way a test can fail."""

    error_signature: str = Field(..., description="Hash of error message/stack trace")
    frequency: float = Field(..., ge=0, le=1, description="Proportion of failures with this signature")
    sample_error: str = Field(..., description="Example error message")


class FlakyTestRecord(BaseModel):
    """Tracks tests identified as flaky with statistical confidence data."""

    record_id: UUID = Field(default_factory=uuid4, description="Unique identifier")
    test_id: str = Field(..., description="Reference to TestCase")
    detected_at: datetime = Field(
        default_factory=datetime.utcnow, description="When flakiness was first detected"
    )
    last_flake_at: datetime = Field(
        default_factory=datetime.utcnow, description="Most recent flaky execution"
    )
    confidence: float = Field(..., ge=0, le=1, description="Statistical confidence in flakiness")
    pass_rate: float = Field(..., ge=0, le=1, description="Proportion of passing executions")
    sample_size: int = Field(..., ge=10, description="Number of executions analyzed")
    consecutive_passes: int = Field(default=0, ge=0, description="Consecutive passes since last failure")
    status: FlakyStatus = Field(default=FlakyStatus.ACTIVE_FLAKY, description="Flaky test status")
    execution_pattern: list[bool] = Field(
        default_factory=list, description="Last N execution results (True=pass)"
    )
    failure_modes: list[FailureMode] = Field(
        default_factory=list, description="Distinct ways test fails"
    )
    investigation_notes: Optional[str] = Field(None, description="Developer notes on flakiness cause")


# ============================================================================
# Deployment Validation Models
# ============================================================================


class ConfigCheck(BaseModel):
    """Environment variable validation result."""

    variable_name: str = Field(..., description="Environment variable name")
    is_present: bool = Field(..., description="Whether variable is set")
    is_valid: bool = Field(..., description="Whether value passes validation")
    error_message: Optional[str] = Field(None, description="Validation error if any")


class SecretRisk(BaseModel):
    """Potential hardcoded secret detection."""

    file: str = Field(..., description="File containing potential secret")
    line: int = Field(..., ge=1, description="Line number")
    secret_type: SecretType = Field(..., description="Type of secret detected")
    severity: Severity = Field(..., description="Risk severity")
    matched_pattern: str = Field(..., description="Pattern that triggered detection")


class DependencyConflict(BaseModel):
    """Version conflict between dependencies."""

    package: str = Field(..., description="Package name")
    required_version: str = Field(..., description="Version required by dependent")
    installed_version: str = Field(..., description="Currently installed version")
    conflicting_dependent: str = Field(..., description="Which package has conflicting requirement")


class OutdatedPackage(BaseModel):
    """A package that needs updating."""

    package: str = Field(..., description="Package name")
    current_version: str = Field(..., description="Installed version")
    latest_version: str = Field(..., description="Latest available version")
    security_update: bool = Field(default=False, description="Whether update fixes security issues")


class ResourceCheck(BaseModel):
    """Resource requirement validation."""

    resource_type: ResourceType = Field(..., description="Type of resource")
    required: str = Field(..., description="Required resource (e.g., '4 cores', '8GB RAM')")
    available: str = Field(..., description="Available resource in environment")
    sufficient: bool = Field(..., description="Whether requirements met")


class PlatformCheck(BaseModel):
    """Platform compatibility check."""

    check_type: PlatformCheckType = Field(..., description="Type of platform check")
    required: str = Field(..., description="Required platform (e.g., 'Linux', 'Python 3.11+')")
    actual: str = Field(..., description="Detected platform")
    compatible: bool = Field(..., description="Whether compatible")


class ServiceCheck(BaseModel):
    """External service availability check."""

    service_name: str = Field(..., description="External service name (e.g., 'database', 'redis')")
    endpoint: str = Field(..., description="Service endpoint/URL")
    is_available: bool = Field(..., description="Whether service is reachable")
    response_time_ms: Optional[float] = Field(None, ge=0, description="Ping response time")


class ConfigurationResults(BaseModel):
    """Configuration validation results."""

    status: CheckStatus = Field(default=CheckStatus.PASSED, description="Config validation status")
    required_variables: list[ConfigCheck] = Field(
        default_factory=list, description="Env var validation"
    )
    secrets_exposure_risks: list[SecretRisk] = Field(
        default_factory=list, description="Hardcoded secret detection"
    )
    schema_validation: dict = Field(default_factory=dict, description="JSON schema validation")


class DependencyResults(BaseModel):
    """Dependency check results."""

    status: CheckStatus = Field(default=CheckStatus.PASSED, description="Dependency check status")
    conflicts: list[DependencyConflict] = Field(default_factory=list, description="Version conflicts")
    vulnerabilities: list[Vulnerability] = Field(
        default_factory=list, description="Security vulnerabilities"
    )
    outdated_packages: list[OutdatedPackage] = Field(
        default_factory=list, description="Packages needing updates"
    )


class CompatibilityResults(BaseModel):
    """Environment compatibility results."""

    status: CheckStatus = Field(
        default=CheckStatus.PASSED, description="Compatibility check status"
    )
    resource_checks: list[ResourceCheck] = Field(
        default_factory=list, description="CPU/memory requirements"
    )
    platform_compatibility: list[PlatformCheck] = Field(
        default_factory=list, description="OS/runtime checks"
    )
    service_availability: list[ServiceCheck] = Field(
        default_factory=list, description="External service checks"
    )


class DeploymentChecklist(BaseModel):
    """Collection of pre-deployment validation checks."""

    checklist_id: UUID = Field(default_factory=uuid4, description="Unique identifier")
    run_id: UUID = Field(..., description="Foreign key to ValidationRun")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="When deployment validation ran"
    )
    status: CheckStatus = Field(default=CheckStatus.PASSED, description="Overall status")
    configuration: ConfigurationResults = Field(
        default_factory=ConfigurationResults, description="Config validation results"
    )
    dependencies: DependencyResults = Field(
        default_factory=DependencyResults, description="Dependency check results"
    )
    compatibility: CompatibilityResults = Field(
        default_factory=CompatibilityResults, description="Environment compatibility"
    )


# ============================================================================
# Validation Run Model (Top-Level)
# ============================================================================


class ValidationRun(BaseModel):
    """A single execution of the complete validation pipeline."""

    run_id: UUID = Field(default_factory=uuid4, description="Unique identifier for this run")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow, description="When the validation started"
    )
    trigger_source: TriggerSource = Field(..., description="How the validation was triggered")
    status: ValidationStatus = Field(
        default=ValidationStatus.PENDING, description="Current validation status"
    )
    started_at: datetime = Field(default_factory=datetime.utcnow, description="When validation started")
    completed_at: Optional[datetime] = Field(None, description="When validation finished")
    duration_seconds: Optional[float] = Field(None, ge=0, description="Total execution time")
    repository: str = Field(..., description="Repository being validated")
    branch: str = Field(..., description="Git branch name")
    commit_sha: str = Field(..., description="Git commit hash")
    user: Optional[str] = Field(None, description="User who triggered validation")

    # Related reports
    quality_report: Optional[CodeQualityReport] = Field(
        None, description="Code quality validation results"
    )
    test_results: list[TestSuiteResult] = Field(
        default_factory=list, description="Test suite results (unit/integration/e2e)"
    )
    deployment_checklist: Optional[DeploymentChecklist] = Field(
        None, description="Pre-deployment validation results"
    )

    @field_validator("completed_at")
    @classmethod
    def validate_completed_at(cls, v: Optional[datetime], info) -> Optional[datetime]:
        """Ensure completed_at is after started_at."""
        if v is not None:
            started_at = info.data.get("started_at")
            if started_at and v < started_at:
                raise ValueError("completed_at must be after started_at")
        return v

    def start(self) -> None:
        """Mark validation as started."""
        self.status = ValidationStatus.RUNNING
        self.started_at = datetime.utcnow()

    def complete(self, status: ValidationStatus) -> None:
        """Mark validation as completed."""
        self.status = status
        self.completed_at = datetime.utcnow()
        if self.started_at:
            self.duration_seconds = (self.completed_at - self.started_at).total_seconds()

    def fail(self) -> None:
        """Mark validation as failed."""
        self.complete(ValidationStatus.FAILED)

    def succeed(self) -> None:
        """Mark validation as passed."""
        self.complete(ValidationStatus.PASSED)


# ============================================================================
# API Models (Request/Response)
# ============================================================================


class ValidationRequest(BaseModel):
    """Request to start a validation run."""

    path: str = Field(..., description="Path to project to validate")
    repository: Optional[str] = Field(None, description="Repository name (auto-detected if omitted)")
    branch: Optional[str] = Field(None, description="Branch name (auto-detected if omitted)")
    commit_sha: Optional[str] = Field(None, description="Commit SHA (auto-detected if omitted)")
    trigger_source: TriggerSource = Field(
        default=TriggerSource.MANUAL, description="Trigger source"
    )
    user: Optional[str] = Field(None, description="User triggering validation")
    run_quality: bool = Field(default=True, description="Run code quality checks")
    run_tests: bool = Field(default=True, description="Run test suites")
    run_deployment: bool = Field(default=True, description="Run deployment checks")


class ValidationSummary(BaseModel):
    """Summary of a validation run for API responses."""

    run_id: UUID = Field(..., description="Validation run ID")
    status: ValidationStatus = Field(..., description="Current status")
    timestamp: datetime = Field(..., description="When validation started")
    duration_seconds: Optional[float] = Field(None, description="Total duration")
    quality_passed: Optional[bool] = Field(None, description="Quality checks passed")
    tests_passed: Optional[bool] = Field(None, description="Tests passed")
    deployment_passed: Optional[bool] = Field(None, description="Deployment checks passed")



# ============================================================================
# Type Aliases for backward compatibility
# ============================================================================

# These aliases provide simplified names used in tests and other modules
QualityReport = CodeQualityReport
QualityIssue = Violation
IssueSeverity = Severity
TestResult = TestSuiteResult
SuiteType = TestSuiteType
TestFailure = TestCaseResult
FlakyTestInfo = FlakyTestRecord
ConfigurationCheck = ConfigCheck
DependencyCheck = ConfigCheck
CompatibilityCheck = ConfigCheck
