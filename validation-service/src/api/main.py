"""FastAPI application for the validation service.

This module provides the REST API for running validations, viewing results,
and managing flaky tests.
"""

import logging
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from src.core.config import get_settings
from src.core.logging import setup_logging


logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    """Application lifespan handler."""
    # Startup
    logger.info("Starting Validation Service API")
    setup_logging()

    # Register pipeline handlers
    from src.quality.validator import setup_quality_pipeline
    from src.testing.pipeline_integration import setup_test_pipeline
    from src.deployment.pipeline_integration import setup_deployment_pipeline

    setup_quality_pipeline()
    setup_test_pipeline()
    setup_deployment_pipeline()

    yield

    # Shutdown
    logger.info("Shutting down Validation Service API")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application.

    Returns:
        Configured FastAPI application
    """
    settings = get_settings()

    app = FastAPI(
        title="Validation Service API",
        description="Code quality, testing, and deployment validation",
        version="1.0.0",
        lifespan=lifespan,
        docs_url="/docs" if settings.environment != "production" else None,
        redoc_url="/redoc" if settings.environment != "production" else None,
    )

    # Configure CORS
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],  # Configure appropriately for production
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    # Include routers
    from src.api.endpoints.validation import router as validation_router
    from src.api.endpoints.results import router as results_router
    from src.api.endpoints.history import router as history_router
    from src.api.endpoints.flaky_tests import router as flaky_tests_router
    from src.api.endpoints.health import router as health_router

    app.include_router(health_router, prefix="/api/v1", tags=["Health"])
    app.include_router(validation_router, prefix="/api/v1", tags=["Validation"])
    app.include_router(results_router, prefix="/api/v1", tags=["Results"])
    app.include_router(history_router, prefix="/api/v1", tags=["History"])
    app.include_router(flaky_tests_router, prefix="/api/v1", tags=["Flaky Tests"])

    return app


# Create the application instance
app = create_app()
