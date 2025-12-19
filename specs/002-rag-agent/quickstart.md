# Quickstart: RAG Agent with OpenAI Agents SDK

**Feature**: 002-rag-agent
**Date**: 2025-12-17
**Related Plan**: [plan.md](./plan.md)

## Overview

This quickstart guide shows how to implement and run a RAG (Retrieval-Augmented Generation) agent using OpenAI Agents SDK integrated with FastAPI, Cohere embeddings, and Qdrant retrieval. The agent accepts natural language queries and returns contextually accurate answers with source attribution.

## Prerequisites

- Python 3.11+
- UV package manager
- OpenAI API key (for agent orchestration and generation)
- Cohere API key (for embedding generation)
- Qdrant Cloud account and API key
- Existing Qdrant collection `rag_embedding` with embedded documents (from 001-qdrant-retrieval-testing)

### Environment Setup

1. **Create environment file**:
   ```bash
   # In backend/ directory
   cat > .env << EOF
   OPENAI_API_KEY=your_openai_api_key_here
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=https://your-cluster-url.qdrant.tech
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_COLLECTION_NAME=rag_embedding
   LOG_LEVEL=INFO
   EOF
   ```

2. **Install dependencies**:
   ```bash
   cd backend
   uv pip install openai-agents-sdk fastapi uvicorn cohere qdrant-client pydantic python-dotenv httpx tenacity
   ```

## Implementation Steps

### Step 1: Configuration Management (C003)

Create `backend/config.py` to load and validate environment variables:

```python
from pydantic_settings import BaseSettings
from pydantic import Field

class Settings(BaseSettings):
    openai_api_key: str = Field(..., env="OPENAI_API_KEY")
    cohere_api_key: str = Field(..., env="COHERE_API_KEY")
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_collection_name: str = Field(default="rag_embedding", env="QDRANT_COLLECTION_NAME")
    log_level: str = Field(default="INFO", env="LOG_LEVEL")

    class Config:
        env_file = ".env"
        case_sensitive = False

settings = Settings()
```

### Step 2: Pydantic Models (C002)

Create `backend/models.py` with request/response models:

```python
from pydantic import BaseModel, Field
from typing import List, Optional

class AgentRequest(BaseModel):
    query: str = Field(..., min_length=1, max_length=1000)
    top_k: int = Field(default=5, ge=1, le=10)
    include_sources: bool = Field(default=True)

class SourceChunk(BaseModel):
    content: str
    url: str
    chunk_id: str
    document_id: str
    similarity_score: float = Field(ge=0.0, le=1.0)
    position: int = Field(ge=1)

class ResponseMetadata(BaseModel):
    response_time_ms: float = Field(ge=0)
    chunk_count: int = Field(ge=0)
    embedding_time_ms: float = Field(ge=0)
    retrieval_time_ms: float = Field(ge=0)
    generation_time_ms: float = Field(ge=0)

class AgentResponse(BaseModel):
    answer: str
    sources: List[SourceChunk]
    metadata: ResponseMetadata

class ErrorResponse(BaseModel):
    error: str
    message: str
    detail: Optional[str] = None
    status_code: int = Field(ge=400, le=599)
```

### Step 3: Retrieval Service Integration (C004)

Create `backend/retrieval.py` to wrap existing Qdrant retrieval:

```python
from typing import List, Dict, Any
from main import retrieve  # Import from existing implementation

async def retrieve_documents(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve relevant documents from Qdrant.
    Wrapper around existing retrieve() function from 001-qdrant-retrieval-testing.
    """
    # Call existing retrieve function
    results = retrieve(query, top_k=top_k, collection_name="rag_embedding")

    # Format for agent consumption
    return [{
        "content": r.content,
        "url": r.url,
        "chunk_id": r.chunk_id,
        "document_id": r.document_id,
        "similarity_score": r.similarity_score,
        "position": r.position
    } for r in results]
```

### Step 4: OpenAI Agent with Retrieval Tool (C005, C006)

Create `backend/rag_agent.py` with agent and tool definition:

```python
from openai import OpenAI
from openai.agents import Agent, tool
from typing import List, Dict
from retrieval import retrieve_documents as retrieve_docs
from config import settings

# Initialize OpenAI client
client = OpenAI(api_key=settings.openai_api_key)

# Define retrieval tool
@tool
def retrieve_documents(query: str, top_k: int = 5) -> List[Dict]:
    """
    Retrieve relevant documents from the knowledge base using vector similarity search.

    Args:
        query: User's natural language question
        top_k: Number of documents to retrieve (default: 5, max: 10)

    Returns:
        List of relevant documents with content, URLs, and similarity scores
    """
    import asyncio
    results = asyncio.run(retrieve_docs(query, top_k))
    return results

# Create RAG agent
rag_agent = Agent(
    name="RAG Assistant",
    instructions="""You are a helpful AI assistant that answers questions about Physical AI and Humanoid Robotics.

When answering questions:
1. Use the retrieve_documents tool to find relevant information from the knowledge base
2. Synthesize the information from multiple sources into a coherent answer
3. Always cite your sources by referencing the retrieved chunks
4. If no relevant information is found, acknowledge this and suggest refining the query
5. Be concise but comprehensive in your answers
6. Maintain a professional and educational tone
""",
    tools=[retrieve_documents],
    model="gpt-4o",
    client=client
)

async def process_query(query: str, top_k: int = 5) -> Dict:
    """Process a query through the RAG agent."""
    import time
    start_time = time.time()

    # Run the agent
    response = rag_agent.run(query)

    end_time = time.time()
    response_time = (end_time - start_time) * 1000

    return {
        "answer": response.content,
        "response_time_ms": response_time
    }
```

### Step 5: FastAPI Application (C001)

Create `backend/api.py` with the `/ask` endpoint:

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from models import AgentRequest, AgentResponse, ErrorResponse
from rag_agent import process_query
import logging

app = FastAPI(
    title="RAG Agent API",
    description="API for RAG agent using OpenAI Agents SDK",
    version="1.0.0"
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@app.post("/ask", response_model=AgentResponse)
async def ask_question(request: AgentRequest):
    """
    Ask a question to the RAG agent.
    """
    try:
        # Validate request
        if not request.query.strip():
            raise HTTPException(
                status_code=400,
                detail=ErrorResponse(
                    error="validation_error",
                    message="Query cannot be empty",
                    detail="The 'query' field must contain at least one character",
                    status_code=400
                ).dict()
            )

        # Process query
        logger.info(f"Processing query: {request.query[:50]}...")
        result = await process_query(request.query, request.top_k)

        # Format response
        return AgentResponse(
            answer=result["answer"],
            sources=result.get("sources", []),
            metadata=result.get("metadata", {})
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=ErrorResponse(
                error="agent_error",
                message="Failed to process query",
                detail=str(e),
                status_code=500
            ).dict()
        )

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "services": {
            "openai": "connected",
            "cohere": "connected",
            "qdrant": "connected"
        },
        "timestamp": "2025-12-17T12:00:00Z"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

## Running the Application

### Start the API Server

```bash
cd backend
python api.py
```

The server will start on `http://localhost:8000`

### Test the `/ask` Endpoint

```bash
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?",
    "top_k": 5,
    "include_sources": true
  }'
```

**Expected Response**:
```json
{
  "answer": "Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world...",
  "sources": [
    {
      "content": "Physical AI involves sophisticated AI algorithms...",
      "url": "https://hassaanghayas.github.io/physical-ai-robotics-textbook/docs/intro",
      "chunk_id": "intro_chunk_0",
      "document_id": "intro_doc",
      "similarity_score": 0.87,
      "position": 1
    }
  ],
  "metadata": {
    "response_time_ms": 3245.5,
    "chunk_count": 3,
    "embedding_time_ms": 250.0,
    "retrieval_time_ms": 150.0,
    "generation_time_ms": 2800.0
  }
}
```

### Check API Health

```bash
curl "http://localhost:8000/health"
```

## Testing Workflow

### Unit Tests

```bash
cd backend
pytest tests/test_agent.py -v
pytest tests/test_api.py -v
pytest tests/test_retrieval.py -v
```

### Integration Tests

```bash
pytest tests/ -v --integration
```

### Error Handling Tests

```bash
# Test empty query
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{"query": ""}'

# Test query too long
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{"query": "'$(python -c 'print("a" * 1001)')'"}'
```

## Validation Criteria

### Successful Query Processing
- Agent returns a coherent answer based on retrieved context
- Response includes at least 3 source chunks when available
- All source chunks have similarity scores above 0.5 (for real embeddings)
- Response time is under 5 seconds for 95% of queries

### Error Handling
- Empty query returns 400 Bad Request
- Service unavailability returns 503 Service Unavailable
- Retry logic activates for transient failures (max 3 attempts)
- Clear error messages guide users to resolution

### Performance Targets
- P95 response time: < 5 seconds
- Average response time: < 3 seconds
- Support for 50 concurrent users
- Zero unhandled exceptions

## Troubleshooting

### Agent Not Returning Results
- Verify Qdrant collection `rag_embedding` has documents
- Check that Cohere API key is valid and not rate-limited
- Ensure OpenAI API key has sufficient quota
- Review logs for error messages

### Slow Response Times
- Check network latency to Qdrant Cloud
- Verify OpenAI model selection (gpt-4o recommended)
- Monitor individual component times in ResponseMetadata
- Consider reducing top_k if retrieval is slow

### Service Unavailable Errors
- Verify all API keys are correctly configured
- Check service status pages (OpenAI, Cohere, Qdrant)
- Review circuit breaker state (may need manual reset)
- Check logs for specific service errors

## Example Usage

```python
import requests

# Example 1: Basic query
response = requests.post(
    "http://localhost:8000/ask",
    json={
        "query": "What is physical AI?",
        "top_k": 5,
        "include_sources": True
    }
)
print(response.json())

# Example 2: Query without sources
response = requests.post(
    "http://localhost:8000/ask",
    json={
        "query": "Explain humanoid robotics",
        "top_k": 3,
        "include_sources": False
    }
)
print(response.json()["answer"])

# Example 3: Health check
response = requests.get("http://localhost:8000/health")
print(response.json())
```

## Success Metrics

Upon successful implementation:

- ✅ `/ask` endpoint responds with valid JSON for all valid queries
- ✅ Agent successfully orchestrates retrieval → generation workflow
- ✅ Source attribution includes chunk content, URLs, and similarity scores
- ✅ Error handling returns appropriate HTTP status codes (400, 500, 503)
- ✅ Response time under 5 seconds for 95% of requests
- ✅ Retry logic handles transient failures from external APIs
- ✅ All unit and integration tests pass
