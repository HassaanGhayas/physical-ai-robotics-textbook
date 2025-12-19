"""Validation endpoints for running validations."""

import logging
from typing import Optional

from fastapi import APIRouter, BackgroundTasks, HTTPException
from pydantic import BaseModel, Field

from src.core.models import TriggerSource
from src.core.pipeline import PipelineConfig, get_pipeline


logger = logging.getLogger(__name__)
router = APIRouter()


class ValidationRequest(BaseModel):
    """Request model for validation."""

    project_path: str = Field(..., description="Path to project directory")
    repository: str = Field(..., description="Repository name")
    branch: str = Field(..., description="Git branch name")
    commit_sha: str = Field(..., description="Git commit SHA")
    run_quality: bool = Field(default=True, description="Run quality checks")
    run_tests: bool = Field(default=True, description="Run test suites")
    run_deployment: bool = Field(default=True, description="Run deployment checks")
    fail_fast: bool = Field(default=False, description="Stop on first failure")
    timeout_minutes: int = Field(default=15, description="Pipeline timeout")


class ValidationResponse(BaseModel):
    """Response model for validation."""

    run_id: str
    status: str
    message: str


async def _run_validation_async(request: ValidationRequest) -> None:
    """Run validation in background."""
    pipeline = get_pipeline()
    config = PipelineConfig(
        run_quality=request.run_quality,
        run_tests=request.run_tests,
        run_deployment=request.run_deployment,
        fail_fast=request.fail_fast,
        timeout_minutes=request.timeout_minutes,
    )

    try:
        await pipeline.run(
            project_path=request.project_path,
            repository=request.repository,
            branch=request.branch,
            commit_sha=request.commit_sha,
            trigger_source=TriggerSource.API,
            config=config,
        )
    except Exception as e:
        logger.error(f"Validation failed: {e}")


@router.post("/validate", response_model=ValidationResponse)
async def run_validation(
    request: ValidationRequest,
    background_tasks: BackgroundTasks,
):
    """Run full validation pipeline."""
    pipeline = get_pipeline()
    config = PipelineConfig(
        run_quality=request.run_quality,
        run_tests=request.run_tests,
        run_deployment=request.run_deployment,
        fail_fast=request.fail_fast,
        timeout_minutes=request.timeout_minutes,
    )

    # Start validation
    run = await pipeline.run(
        project_path=request.project_path,
        repository=request.repository,
        branch=request.branch,
        commit_sha=request.commit_sha,
        trigger_source=TriggerSource.API,
        config=config,
    )

    return ValidationResponse(
        run_id=str(run.run_id),
        status=run.status.value,
        message=f"Validation {run.status.value}",
    )


@router.post("/validate/quality", response_model=ValidationResponse)
async def run_quality_validation(request: ValidationRequest):
    """Run only code quality validation."""
    pipeline = get_pipeline()

    run = await pipeline.run_quality_only(
        project_path=request.project_path,
        repository=request.repository,
        branch=request.branch,
        commit_sha=request.commit_sha,
    )

    return ValidationResponse(
        run_id=str(run.run_id),
        status=run.status.value,
        message="Quality validation completed",
    )


@router.post("/validate/tests", response_model=ValidationResponse)
async def run_test_validation(request: ValidationRequest):
    """Run only test suites."""
    pipeline = get_pipeline()

    run = await pipeline.run_tests_only(
        project_path=request.project_path,
        repository=request.repository,
        branch=request.branch,
        commit_sha=request.commit_sha,
    )

    return ValidationResponse(
        run_id=str(run.run_id),
        status=run.status.value,
        message="Test execution completed",
    )


@router.post("/validate/deployment", response_model=ValidationResponse)
async def run_deployment_validation(request: ValidationRequest):
    """Run only deployment validation."""
    pipeline = get_pipeline()

    run = await pipeline.run_deployment_only(
        project_path=request.project_path,
        repository=request.repository,
        branch=request.branch,
        commit_sha=request.commit_sha,
    )

    return ValidationResponse(
        run_id=str(run.run_id),
        status=run.status.value,
        message="Deployment validation completed",
    )
