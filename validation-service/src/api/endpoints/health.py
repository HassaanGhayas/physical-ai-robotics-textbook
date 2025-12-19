"""Health check endpoints."""

import sys
import time
from typing import Optional

from fastapi import APIRouter, Query
from pydantic import BaseModel

router = APIRouter()

# Track startup time for uptime calculation
_startup_time = time.time()
_request_count = 0


class HealthResponse(BaseModel):
    """Health check response."""

    status: str
    service: str


class DetailedHealthResponse(BaseModel):
    """Detailed health check response."""

    status: str
    service: str
    components: dict


class ReadinessResponse(BaseModel):
    """Readiness check response."""

    ready: bool


class LivenessResponse(BaseModel):
    """Liveness check response."""

    alive: bool


class MetricsResponse(BaseModel):
    """Metrics response."""

    uptime_seconds: float
    total_requests: int


class VersionResponse(BaseModel):
    """Version response."""

    version: str
    python_version: str


@router.get("/health")
async def health_check(details: Optional[bool] = Query(False)):
    """Check API health status."""
    global _request_count
    _request_count += 1

    if details:
        return DetailedHealthResponse(
            status="healthy",
            service="validation-service",
            components={
                "api": "healthy",
                "storage": "healthy",
            },
        )

    return HealthResponse(status="healthy", service="validation-service")


@router.get("/ready")
async def readiness_check():
    """Check API readiness."""
    return ReadinessResponse(ready=True)


@router.get("/health/ready")
async def readiness_check_alt():
    """Check API readiness (alternate path)."""
    return ReadinessResponse(ready=True)


@router.get("/health/live")
async def liveness_check():
    """Check API liveness."""
    return LivenessResponse(alive=True)


@router.get("/health/metrics")
async def metrics_endpoint():
    """Get health metrics."""
    global _request_count
    _request_count += 1

    return MetricsResponse(
        uptime_seconds=time.time() - _startup_time,
        total_requests=_request_count,
    )


@router.get("/health/version")
async def version_endpoint():
    """Get version information."""
    return VersionResponse(
        version="1.0.0",
        python_version=sys.version,
    )
