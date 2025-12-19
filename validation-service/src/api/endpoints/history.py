"""History endpoints for viewing validation history."""

import logging
from typing import Optional

from fastapi import APIRouter, HTTPException, Query

from src.core.storage import get_storage
from src.testing.history import get_history_tracker


logger = logging.getLogger(__name__)
router = APIRouter()


@router.get("/history")
async def get_validation_history(
    limit: int = Query(default=10, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
):
    """Get validation run history."""
    storage = get_storage()

    # Get all runs (simplified - in production would use proper pagination)
    runs = storage.list("runs/")

    # Sort by timestamp descending
    runs_data = []
    for run_key in runs:
        run_id = run_key.replace("runs/", "")
        try:
            from uuid import UUID
            run = storage.get_run(UUID(run_id))
            if run:
                runs_data.append({
                    "run_id": str(run.run_id),
                    "status": run.status.value,
                    "repository": run.repository,
                    "branch": run.branch,
                    "commit_sha": run.commit_sha,
                    "started_at": run.started_at.isoformat() if run.started_at else None,
                    "duration_seconds": run.duration_seconds,
                })
        except Exception:
            continue

    # Sort and paginate
    runs_data.sort(key=lambda x: x.get("started_at") or "", reverse=True)
    total = len(runs_data)
    paginated = runs_data[offset:offset + limit]

    return {
        "total": total,
        "limit": limit,
        "offset": offset,
        "runs": paginated,
    }


@router.get("/history/tests/{test_id}")
async def get_test_history(
    test_id: str,
    limit: int = Query(default=10, ge=1, le=100),
):
    """Get execution history for a specific test."""
    tracker = get_history_tracker()
    test_case = tracker.get_test_history(test_id, limit=limit)

    if not test_case:
        raise HTTPException(status_code=404, detail=f"Test {test_id} not found")

    return {
        "test_id": test_case.test_id,
        "test_name": test_case.test_name,
        "test_type": test_case.test_type.value,
        "file_path": test_case.file_path,
        "target_functionality": test_case.target_functionality,
        "execution_history": [
            {
                "run_id": str(record.run_id),
                "timestamp": record.timestamp.isoformat(),
                "status": record.status.value,
                "duration_seconds": record.duration_seconds,
                "commit_sha": record.commit_sha,
            }
            for record in test_case.execution_history
        ],
        "average_duration_seconds": test_case.average_duration_seconds,
        "flakiness_score": test_case.flakiness_score,
        "is_flaky": test_case.is_flaky,
        "last_execution": test_case.last_execution.isoformat() if test_case.last_execution else None,
    }
