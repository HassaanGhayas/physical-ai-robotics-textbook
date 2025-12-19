"""Results endpoints for viewing validation results."""

import logging
from typing import Optional
from uuid import UUID

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from src.core.storage import get_storage


logger = logging.getLogger(__name__)
router = APIRouter()


@router.get("/results/{run_id}")
async def get_results(run_id: UUID):
    """Get full validation results for a run."""
    storage = get_storage()
    run = storage.get_run(run_id)

    if not run:
        raise HTTPException(status_code=404, detail=f"Run {run_id} not found")

    return {
        "run_id": str(run.run_id),
        "status": run.status.value,
        "trigger_source": run.trigger_source.value,
        "repository": run.repository,
        "branch": run.branch,
        "commit_sha": run.commit_sha,
        "started_at": run.started_at.isoformat() if run.started_at else None,
        "completed_at": run.completed_at.isoformat() if run.completed_at else None,
        "duration_seconds": run.duration_seconds,
        "quality_report": run.quality_report.model_dump() if run.quality_report else None,
        "test_results": [r.model_dump() for r in run.test_results] if run.test_results else [],
        "deployment_checklist": run.deployment_checklist.model_dump() if run.deployment_checklist else None,
    }


@router.get("/results/{run_id}/status")
async def get_status(run_id: UUID):
    """Get current status of a validation run."""
    storage = get_storage()
    run = storage.get_run(run_id)

    if not run:
        raise HTTPException(status_code=404, detail=f"Run {run_id} not found")

    # Determine current stage
    stages_completed = []
    current_stage = None

    if run.quality_report:
        stages_completed.append("quality")
    if run.test_results:
        if any(r.suite_type.value == "unit" for r in run.test_results):
            stages_completed.append("unit_tests")
        if any(r.suite_type.value == "integration" for r in run.test_results):
            stages_completed.append("integration_tests")
        if any(r.suite_type.value == "e2e" for r in run.test_results):
            stages_completed.append("e2e_tests")
    if run.deployment_checklist:
        stages_completed.append("deployment")

    elapsed_seconds = run.duration_seconds or 0

    return {
        "run_id": str(run.run_id),
        "status": run.status.value,
        "progress": {
            "completed_stages": stages_completed,
            "current_stage": current_stage,
        },
        "elapsed_seconds": elapsed_seconds,
    }
