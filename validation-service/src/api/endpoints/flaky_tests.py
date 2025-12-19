"""Flaky tests endpoints for managing flaky test records."""

import logging
from datetime import datetime
from typing import Optional

from fastapi import APIRouter, HTTPException, Query
from pydantic import BaseModel

from src.core.storage import get_storage
from src.core.models import FlakyStatus


logger = logging.getLogger(__name__)
router = APIRouter()


class FlakyTestUpdate(BaseModel):
    """Request model for updating flaky test."""

    status: Optional[str] = None
    notes: Optional[str] = None


@router.get("/flaky-tests")
async def list_flaky_tests(
    status: Optional[str] = Query(default=None, description="Filter by status"),
    min_confidence: float = Query(default=0.0, ge=0.0, le=1.0),
    limit: int = Query(default=50, ge=1, le=100),
):
    """Get all detected flaky tests."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    # Filter by status
    if status:
        flaky_tests = [t for t in flaky_tests if t.status.value == status]

    # Filter by confidence
    flaky_tests = [t for t in flaky_tests if t.confidence >= min_confidence]

    # Limit results
    flaky_tests = flaky_tests[:limit]

    # Calculate statistics
    all_flaky = storage.get_active_flaky_tests()
    stats = {
        "active_flaky": sum(1 for t in all_flaky if t.status == FlakyStatus.ACTIVE_FLAKY),
        "false_positive": sum(1 for t in all_flaky if t.status == FlakyStatus.FALSE_POSITIVE),
        "resolved": sum(1 for t in all_flaky if t.status == FlakyStatus.RESOLVED),
    }

    return {
        "tests": [
            {
                "test_id": t.test_id,
                "detected_at": t.detected_at.isoformat(),
                "last_flake_at": t.last_flake_at.isoformat() if t.last_flake_at else None,
                "confidence": t.confidence,
                "pass_rate": t.pass_rate,
                "sample_size": t.sample_size,
                "consecutive_passes": t.consecutive_passes,
                "status": t.status.value,
                "failure_modes": [
                    {
                        "error_signature": fm.error_signature,
                        "frequency": fm.frequency,
                        "sample_error": fm.sample_error,
                    }
                    for fm in t.failure_modes
                ],
            }
            for t in flaky_tests
        ],
        "total_count": len(flaky_tests),
        "statistics": stats,
    }


@router.get("/flaky-tests/stats")
async def get_flaky_stats():
    """Get flaky test statistics."""
    storage = get_storage()
    all_flaky = storage.get_active_flaky_tests()

    return {
        "total": len(all_flaky),
        "active": sum(1 for t in all_flaky if t.status == FlakyStatus.ACTIVE_FLAKY),
        "resolved": sum(1 for t in all_flaky if t.status == FlakyStatus.RESOLVED),
        "quarantined": sum(1 for t in all_flaky if t.status == FlakyStatus.FALSE_POSITIVE),
    }


@router.get("/flaky-tests/{test_id}")
async def get_flaky_test(test_id: str):
    """Get details for a specific flaky test."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    test = next((t for t in flaky_tests if t.test_id == test_id), None)

    if not test:
        raise HTTPException(status_code=404, detail=f"Flaky test {test_id} not found")

    return {
        "test_id": test.test_id,
        "detected_at": test.detected_at.isoformat(),
        "last_flake_at": test.last_flake_at.isoformat() if test.last_flake_at else None,
        "confidence": test.confidence,
        "pass_rate": test.pass_rate,
        "sample_size": test.sample_size,
        "consecutive_passes": test.consecutive_passes,
        "status": test.status.value,
        "execution_pattern": test.execution_pattern,
        "failure_modes": [
            {
                "error_signature": fm.error_signature,
                "frequency": fm.frequency,
                "sample_error": fm.sample_error,
            }
            for fm in test.failure_modes
        ],
    }


@router.patch("/flaky-tests/{test_id}")
async def update_flaky_test(test_id: str, update: FlakyTestUpdate):
    """Update the status of a flaky test."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    test = next((t for t in flaky_tests if t.test_id == test_id), None)

    if not test:
        raise HTTPException(status_code=404, detail=f"Flaky test {test_id} not found")

    # Update status
    if update.status:
        try:
            test.status = FlakyStatus(update.status)
        except ValueError:
            raise HTTPException(
                status_code=400,
                detail=f"Invalid status: {update.status}. Valid values: {[s.value for s in FlakyStatus]}"
            )

    # Save updated record
    storage.save(f"flaky_tests/{test_id}", test.model_dump(mode="json"))

    return {
        "test_id": test.test_id,
        "status": test.status.value,
        "updated_at": datetime.utcnow().isoformat(),
        "notes": update.notes,
    }


@router.post("/flaky-tests/{test_id}/acknowledge")
async def acknowledge_flaky_test(test_id: str):
    """Acknowledge a flaky test."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    test = next((t for t in flaky_tests if t.test_id == test_id), None)

    if not test:
        raise HTTPException(status_code=404, detail=f"Flaky test {test_id} not found")

    return {
        "test_id": test.test_id,
        "acknowledged": True,
        "acknowledged_at": datetime.utcnow().isoformat(),
    }


@router.post("/flaky-tests/{test_id}/quarantine")
async def quarantine_flaky_test(test_id: str):
    """Quarantine a flaky test."""
    storage = get_storage()
    flaky_tests = storage.get_active_flaky_tests()

    test = next((t for t in flaky_tests if t.test_id == test_id), None)

    if not test:
        raise HTTPException(status_code=404, detail=f"Flaky test {test_id} not found")

    test.status = FlakyStatus.FALSE_POSITIVE
    storage.save(f"flaky_tests/{test_id}", test.model_dump(mode="json"))

    return {
        "test_id": test.test_id,
        "quarantined": True,
        "quarantined_at": datetime.utcnow().isoformat(),
    }
