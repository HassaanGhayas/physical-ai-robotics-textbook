"""
FastAPI application for RAG Agent.
Exposes /ask endpoint with comprehensive error handling.
"""
import logging
import asyncio
import uuid
from datetime import datetime
from fastapi import FastAPI, HTTPException, Request, status
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from tenacity import retry, stop_after_attempt, wait_exponential, retry_if_exception_type
from pybreaker import CircuitBreaker, CircuitBreakerError
import time

from models import AgentRequest, AgentResponse, ErrorResponse, ErrorType, HealthResponse
from rag_agent import process_query
from config import settings

# Configure logging
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG agent with Cohere embeddings and Qdrant retrieval",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Local development
        "https://hassaanghayas.github.io"  # Production deployment
    ],
    allow_credentials=False,
    allow_methods=["POST", "GET", "OPTIONS"],
    allow_headers=["Content-Type"],
    max_age=600,
)


# Circuit breakers for external services
cohere_breaker = CircuitBreaker(
    fail_max=settings.circuit_breaker_threshold,
    reset_timeout=60
)

qdrant_breaker = CircuitBreaker(
    fail_max=settings.circuit_breaker_threshold,
    reset_timeout=60
)


# Retry decorator with exponential backoff
def with_retry(func):
    """Decorator to add retry logic with exponential backoff."""
    return retry(
        stop=stop_after_attempt(settings.max_retries),
        wait=wait_exponential(multiplier=1, min=1, max=10),
        retry=retry_if_exception_type((TimeoutError, ConnectionError)),
        reraise=True
    )(func)


# Exception handlers
@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions with structured error responses."""
    logger.warning(f"HTTP exception: {exc.status_code} - {exc.detail}")
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": "http_error",
            "message": str(exc.detail),
            "detail": None,
            "status_code": exc.status_code
        }
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle all other exceptions."""
    logger.error(f"Unhandled exception: {str(exc)}", exc_info=True)
    return JSONResponse(
        status_code=500,
        content={
            "error": ErrorType.UNKNOWN_ERROR,
            "message": "Internal server error",
            "detail": str(exc) if settings.log_level == "DEBUG" else None,
            "status_code": 500
        }
    )


# Middleware for request logging
@app.middleware("http")
async def log_requests(request: Request, call_next):
    """Log all incoming requests with request ID."""
    request_id = str(uuid.uuid4())
    request.state.request_id = request_id

    logger.info(f"Request {request_id}: {request.method} {request.url.path}")

    start_time = time.time()
    response = await call_next(request)
    process_time = (time.time() - start_time) * 1000

    logger.info(f"Request {request_id} completed in {process_time:.2f}ms with status {response.status_code}")

    return response


@app.post("/ask", response_model=AgentResponse, status_code=200)
async def ask_question(request: AgentRequest):
    """
    Ask a question to the RAG agent.

    The agent will:
    1. Retrieve relevant documents from the knowledge base
    2. Generate a contextually accurate answer
    3. Provide source attribution with similarity scores

    Args:
        request: AgentRequest with query, top_k, and include_sources

    Returns:
        AgentResponse with answer, sources, and metadata

    Raises:
        HTTPException: For validation errors (400), service errors (503), or internal errors (500)
    """
    request_start = time.time()

    try:
        # Input validation
        if not request.query or not request.query.strip():
            raise HTTPException(
                status_code=400,
                detail="Query cannot be empty"
            )

        if len(request.query) > 1000:
            raise HTTPException(
                status_code=400,
                detail=f"Query exceeds maximum length of 1000 characters (got {len(request.query)})"
            )

        if request.top_k < 1 or request.top_k > 10:
            raise HTTPException(
                status_code=400,
                detail=f"top_k must be between 1 and 10 (got {request.top_k})"
            )

        # Process query with timeout
        try:
            logger.info(f"Processing query: '{request.query[:50]}...' with top_k={request.top_k}")

            # Apply timeout to prevent hanging requests
            response = await asyncio.wait_for(
                process_query(
                    query=request.query,
                    top_k=request.top_k,
                    include_sources=request.include_sources
                ),
                timeout=settings.rag_agent_timeout
            )

            # Log success
            logger.info(f"Query processed successfully in {response.metadata.response_time_ms:.2f}ms with {response.metadata.chunk_count} chunks")

            return response

        except asyncio.TimeoutError:
            logger.error(f"Query timed out after {settings.rag_agent_timeout}s")
            raise HTTPException(
                status_code=504,
                detail=f"Request timed out after {settings.rag_agent_timeout} seconds"
            )

        except CircuitBreakerError as e:
            logger.error(f"Circuit breaker open: {str(e)}")
            raise HTTPException(
                status_code=503,
                detail="Service temporarily unavailable due to high error rate. Please try again later."
            )

    except HTTPException:
        raise

    except Exception as e:
        logger.error(f"Error processing query: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )


@app.get("/health", response_model=HealthResponse, status_code=200)
async def health_check():
    """
    Health check endpoint.

    Checks connectivity to all external services:
    - Cohere API (for embeddings and generation)
    - Qdrant database (for vector storage)

    Returns:
        HealthResponse with service statuses
    """
    services = {
        "cohere": "unknown",
        "qdrant": "unknown"
    }

    overall_status = "healthy"

    # Check Cohere (via retrieval test)
    try:
        from main import check_cohere_availability
        if check_cohere_availability():
            services["cohere"] = "available"
        else:
            services["cohere"] = "unavailable"
            overall_status = "unhealthy"
    except Exception as e:
        logger.warning(f"Cohere health check failed: {str(e)}")
        services["cohere"] = "unavailable"
        overall_status = "unhealthy"

    # Check Qdrant
    try:
        from main import check_qdrant_availability
        if check_qdrant_availability():
            services["qdrant"] = "available"
        else:
            services["qdrant"] = "unavailable"
            overall_status = "unhealthy"
    except Exception as e:
        logger.warning(f"Qdrant health check failed: {str(e)}")
        services["qdrant"] = "unavailable"
        overall_status = "unhealthy"

    return HealthResponse(
        status=overall_status,
        services=services,
        timestamp=datetime.utcnow().isoformat() + "Z",
        version="1.0.0"
    )


@app.get("/")
async def root():
    """Root endpoint with API information."""
    return {
        "name": "RAG Agent API",
        "version": "1.0.0",
        "endpoints": {
            "/ask": "POST - Ask a question to the RAG agent",
            "/health": "GET - Check service health",
            "/docs": "GET - API documentation"
        }
    }


if __name__ == "__main__":
    import uvicorn
    logger.info("Starting RAG Agent API server...")
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level=settings.log_level.lower())
