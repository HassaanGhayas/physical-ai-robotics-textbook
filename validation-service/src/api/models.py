"""API request/response models."""

from datetime import datetime
from typing import Any, Optional
from uuid import UUID

from pydantic import BaseModel, Field


class ErrorResponse(BaseModel):
    """Standard error response."""

    error: dict = Field(description="Error details")


class ErrorDetail(BaseModel):
    """Error detail structure."""

    code: str
    message: str
    details: Optional[dict] = None


class PaginatedResponse(BaseModel):
    """Base model for paginated responses."""

    total: int
    limit: int
    offset: int


class ValidationSummary(BaseModel):
    """Summary of a validation run."""

    run_id: UUID
    status: str
    repository: str
    branch: str
    commit_sha: str
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    duration_seconds: Optional[float] = None


class QualityReportSummary(BaseModel):
    """Summary of quality report."""

    status: str
    total_violations: int
    error_count: int
    warning_count: int
    max_complexity: int
    line_coverage_percent: float
    quality_gate_passed: bool


class TestResultSummary(BaseModel):
    """Summary of test results."""

    suite_type: str
    status: str
    total_tests: int
    passed_count: int
    failed_count: int
    skipped_count: int
    duration_seconds: float


class DeploymentSummary(BaseModel):
    """Summary of deployment validation."""

    status: str
    configuration_status: str
    dependencies_status: str
    compatibility_status: str
    is_deployable: bool


class WebhookPayload(BaseModel):
    """Webhook notification payload."""

    event: str
    timestamp: datetime
    data: dict


class HealthResponse(BaseModel):
    """Health check response."""

    status: str
    service: str
    version: Optional[str] = None
    uptime_seconds: Optional[float] = None
