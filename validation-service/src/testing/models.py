"""Data models for test execution.

This module provides models specific to test execution functionality,
extending the core models with testing-specific structures.
"""

from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID, uuid4

from pydantic import BaseModel, Field

from src.core.models import TestSuiteType, TestStatus


class TestRunnerStatus(str, Enum):
    """Status of a test runner execution."""

    IDLE = "idle"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    TIMEOUT = "timeout"


@dataclass
class TestRunConfig:
    """Configuration for a test run."""

    project_path: str
    test_type: TestSuiteType
    parallel: bool = True
    timeout_seconds: int = 300
    retries: int = 0
    verbose: bool = False
    coverage: bool = True
    markers: list[str] = field(default_factory=list)
    patterns: list[str] = field(default_factory=list)


class TestExecutionResult(BaseModel):
    """Result from executing a single test."""

    test_id: str = Field(..., description="Unique test identifier")
    test_name: str = Field(..., description="Human-readable test name")
    file_path: str = Field(..., description="Path to test file")
    status: TestStatus = Field(..., description="Test result status")
    duration_seconds: float = Field(..., ge=0, description="Execution time")
    error_message: Optional[str] = Field(None, description="Error message if failed")
    stack_trace: Optional[str] = Field(None, description="Stack trace if failed")
    output: Optional[str] = Field(None, description="Captured stdout/stderr")
    retry_count: int = Field(default=0, ge=0, description="Number of retries")


class TestSuiteExecution(BaseModel):
    """Execution details for a test suite."""

    execution_id: UUID = Field(default_factory=uuid4, description="Unique execution ID")
    run_id: UUID = Field(..., description="Parent validation run ID")
    suite_type: TestSuiteType = Field(..., description="Type of test suite")
    status: TestRunnerStatus = Field(default=TestRunnerStatus.IDLE, description="Runner status")
    started_at: Optional[datetime] = Field(None, description="Start time")
    completed_at: Optional[datetime] = Field(None, description="End time")

    total_tests: int = Field(default=0, ge=0)
    passed: int = Field(default=0, ge=0)
    failed: int = Field(default=0, ge=0)
    skipped: int = Field(default=0, ge=0)
    errors: int = Field(default=0, ge=0)

    results: list[TestExecutionResult] = Field(default_factory=list)
    flaky_tests: list[str] = Field(default_factory=list)

    coverage_percent: float = Field(default=0.0, ge=0, le=100)


class TestDiscoveryResult(BaseModel):
    """Result from test discovery."""

    suite_type: TestSuiteType = Field(..., description="Test suite type")
    total_tests: int = Field(..., ge=0, description="Number of tests found")
    test_files: list[str] = Field(default_factory=list, description="Test file paths")
    test_ids: list[str] = Field(default_factory=list, description="Test identifiers")
    estimated_duration_seconds: float = Field(
        default=0.0, ge=0, description="Estimated execution time"
    )
