# Research: RAG Agent Implementation with OpenAI Agents SDK

**Feature**: 002-rag-agent
**Date**: 2025-12-17
**Research Lead**: Claude
**Related Spec**: [spec.md](./spec.md)

## Executive Summary

This research addresses the implementation of a RAG (Retrieval-Augmented Generation) agent using OpenAI Agents SDK, focusing on tool definitions, multi-service orchestration (Cohere, Qdrant, OpenAI), error handling strategies, and FastAPI integration patterns. The research validates that the OpenAI Agents SDK provides the necessary primitives for sophisticated agent orchestration while maintaining simplicity and production readiness.

## Research Questions

### RQ-001: How to structure agents using OpenAI Agents SDK for RAG applications?

**Background**: Need to understand the fundamental architecture of OpenAI Agents SDK to build a RAG agent that orchestrates retrieval and generation.

**Research**: OpenAI Agents SDK is built on four fundamental primitives:
1. **Agents** - LLMs equipped with instructions and tools
2. **Handoffs** - Enable agents to delegate to other agents
3. **Guardrails** - Provide validation of inputs/outputs
4. **Sessions** - Automatically maintain conversation history

The SDK follows a Python-first approach with minimal abstractions, opinionated defaults, and built-in tracing for visualization and debugging.

**Resolution**: Use a single RAGAgent class with one primary tool (`retrieve_documents`) that handles the complete RAG workflow. The agent will receive instructions to synthesize answers from retrieved context and use the OpenAI Agents SDK's automatic tool calling to invoke retrieval when needed.

### RQ-002: What is the optimal pattern for tool definition and calling in the agent?

**Background**: Need to define how Qdrant retrieval will be exposed as a tool that the OpenAI agent can call.

**Research**: The SDK supports multiple tool definition methods:
- Function Tool Decorator (`@tool`)
- Pre-built Tools (WebSearchTool, FileSearchTool)
- External Tool Registration (MCP-compliant servers)

Key principles:
- Automatic schema generation via Pydantic
- Tool registration with metadata (name, schema, documentation)
- Support for parallel tool calls when enabled

**Resolution**: Use the `@tool` decorator to define a `retrieve_documents` function that:
- Accepts `query` (string) and `top_k` (int) parameters
- Returns structured JSON with document chunks, metadata, and similarity scores
- Integrates with existing Cohere embedding and Qdrant retrieval infrastructure

### RQ-003: How to integrate Cohere embeddings with Qdrant retrieval in the agent workflow?

**Background**: Need to define the integration pattern for combining Cohere (embeddings), Qdrant (retrieval), and OpenAI (generation).

**Research**: Recommended pattern from Cohere + Qdrant documentation:
- Use Cohere `embed-multilingual-v3.0` (1024-dimensional embeddings)
- Specify `input_type="search_query"` for queries
- Specify `input_type="search_document"` for document storage
- Search Qdrant using vector similarity
- Return results as JSON with document metadata

**Resolution**: Implement a retrieval service that:
1. Generates query embedding using Cohere with `input_type="search_query"`
2. Searches Qdrant collection `rag_embedding` using vector similarity
3. Returns formatted results with content, URL, chunk_id, document_id, and similarity scores
4. Reuses existing infrastructure from 001-qdrant-retrieval-testing

### RQ-004: What is the best practice for FastAPI + async agent integration?

**Background**: Need to determine how to expose agent functionality via FastAPI with proper async handling and timeouts.

**Research**: Best practices include:
- Use async/await for all agent calls
- Implement timeouts using `asyncio.wait_for()`
- Proper error boundaries with FastAPI exception handlers
- Structured logging with request IDs

**Resolution**: Create FastAPI endpoint `/ask` that:
- Accepts POST requests with JSON body
- Validates requests using Pydantic models
- Calls agent asynchronously with 5-second timeout
- Returns structured JSON responses
- Handles errors with appropriate HTTP status codes (400, 500, 503)

### RQ-005: What error handling strategy is needed for multi-service orchestration?

**Background**: Need comprehensive error handling for coordinating failures across Cohere, Qdrant, and OpenAI APIs.

**Research**: Industry best practices recommend a 5-layer approach:

**Layer 1: Input Validation**
- Validate query presence, length, and format
- Return 400 Bad Request for invalid inputs

**Layer 2: Retry Logic with Exponential Backoff**
- Use tenacity library for retry decorators
- Configure: stop_after_attempt(3), wait_exponential(multiplier=1, min=1, max=10)
- Retry on transient errors (TimeoutError, ConnectionError, RateLimitError)
- Fail fast on fatal errors

**Layer 3: Circuit Breaker Pattern**
- Use pybreaker library
- Configure: fail_max=5, reset_timeout=60 seconds
- Prevent cascading failures when services degrade
- Open circuit after 5 consecutive failures

**Layer 4: Graceful Degradation**
- Primary: Full RAG pipeline (retrieval + generation)
- Fallback 1: Direct OpenAI generation without retrieval
- Fallback 2: Cached response or error message

**Layer 5: Timeout Management**
- Use `asyncio.wait_for()` for operations
- Set timeout_seconds=5.0 for overall request
- Individual service timeouts: embedding (1s), retrieval (2s), generation (5s)

**Resolution**: Implement all 5 layers with proper error response schema including error type, message, status code, and optional retry_after_seconds.

## Technical Approaches Evaluated

### Approach 1: Single Agent with Retrieval Tool (Selected)
- **Pros**: Simple architecture, single responsibility, easy to test and maintain
- **Cons**: Less flexible for complex multi-turn conversations (out of scope)
- **Verdict**: Selected as primary approach - aligns with stateless requirement and simplicity principle

### Approach 2: Multi-Agent with Handoffs
- **Pros**: More sophisticated, can handle routing to specialized agents
- **Cons**: Added complexity, not needed for single-endpoint RAG
- **Verdict**: Deferred to v2 - overkill for current requirements

### Approach 3: Direct OpenAI API Calls (No SDK)
- **Pros**: More control, no SDK dependency
- **Cons**: Missing built-in tracing, sessions, and orchestration
- **Verdict**: Rejected - SDK provides valuable primitives and simplifies implementation

## Dependencies Resolved

### Dependency 1: OpenAI Agents SDK
- **Status**: Available via pip (`openai-agents-sdk`)
- **Version**: Latest stable (compatible with OpenAI API v1.0+)
- **Capabilities**: Agent creation, tool calling, session management, built-in tracing

### Dependency 2: Cohere + Qdrant Integration
- **Status**: Already implemented in 001-qdrant-retrieval-testing
- **Reuse Strategy**: Wrap existing `retrieve()` function from backend/main.py
- **Compatibility**: 1024-dimensional embeddings from Cohere `embed-multilingual-v3.0`

### Dependency 3: FastAPI Async Patterns
- **Status**: Well-documented and battle-tested
- **Libraries**: FastAPI, uvicorn, httpx, pytest-asyncio
- **Pattern**: Async endpoint → async agent call → structured JSON response

### Dependency 4: Error Handling Libraries
- **Status**: Available via pip
- **Libraries**: tenacity (retries), pybreaker (circuit breaker), structlog (logging)
- **Configuration**: Exponential backoff, circuit breaker thresholds, structured logs

## Key Decisions

### Decision 1: Agent Structure
**Chosen**: Single RAGAgent class with `retrieve_documents` tool
**Pattern**:
```python
@tool
def retrieve_documents(query: str, top_k: int = 5) -> List[Dict]:
    """Retrieve relevant documents from knowledge base."""
    # 1. Generate embedding using Cohere
    # 2. Search Qdrant
    # 3. Format results
    return results
```
**Rationale**: Simplest approach that meets all requirements, easy to test, leverages existing infrastructure

### Decision 2: Integration Pattern
**Chosen**: Sequential orchestration (Query → Embed → Retrieve → Generate)
**Flow**:
1. FastAPI receives request
2. Validate input
3. Agent invokes retrieve_documents tool
4. Tool calls Cohere for embedding
5. Tool searches Qdrant
6. Agent synthesizes answer from context
7. Format response with sources
**Rationale**: Clear data flow, matches RAG pattern, enables proper error handling at each step

### Decision 3: Error Handling Strategy
**Chosen**: 5-layer approach (validation → retry → circuit breaker → degradation → timeout)
**Configuration**:
- Retry: 3 attempts, exponential backoff (1s, 2s, 4s)
- Circuit breaker: Open after 5 failures, reset after 60s
- Timeout: 5s overall, 1s embedding, 2s retrieval, 5s generation
- Degradation: Full RAG → OpenAI only → Error message
**Rationale**: Production-ready resilience, handles all failure modes, provides clear error messages

### Decision 4: Response Format
**Chosen**: Structured JSON with answer, sources, and metadata
**Schema**:
```json
{
  "answer": "...",
  "sources": [{"content": "...", "url": "...", "similarity_score": 0.87}],
  "metadata": {"response_time_ms": 3245, "chunk_count": 3}
}
```
**Rationale**: Meets specification requirements, enables source attribution, provides observability

## Implementation Recommendations

### Recommendation 1: Reuse Existing Infrastructure
- Use existing `retrieve()` function from backend/main.py (001-qdrant-retrieval-testing)
- Leverage existing Cohere client setup and Qdrant connection
- Maintain consistency with existing embedding dimensions (1024)

### Recommendation 2: Modular File Structure
- `api.py`: FastAPI application and endpoints
- `rag_agent.py`: OpenAI agent definition and tool registration
- `models.py`: Pydantic models for validation
- `config.py`: Configuration and environment variables
- `retrieval.py`: Wrapper around existing retrieval functionality

### Recommendation 3: Comprehensive Testing
- Unit tests for each component (agent, tools, API)
- Integration tests for complete workflow
- Error handling tests for all failure modes
- Load tests for 50 concurrent users

### Recommendation 4: Observability
- Structured logging with request IDs
- Performance metrics (response time, chunk count, similarity scores)
- Error tracking (rate, type, recovery)
- Health check endpoint for service monitoring

## Unknowns Resolved

| Unknown | Resolution | Confidence |
|---------|------------|------------|
| OpenAI Agents SDK architecture | Single agent with tool calling pattern | High |
| Tool definition approach | Use `@tool` decorator with Pydantic validation | High |
| Cohere + Qdrant integration | Reuse existing infrastructure, wrap in tool | High |
| FastAPI async pattern | Async endpoint calling async agent with timeout | High |
| Error handling strategy | 5-layer approach with retry, circuit breaker, degradation | High |
| Response format | Structured JSON with answer, sources, metadata | High |
| Performance targets | Sub-5s response achievable with proper async handling | Medium |

## Next Steps

1. Implement data models (AgentRequest, AgentResponse, SourceChunk, ResponseMetadata, ErrorResponse)
2. Create API contracts in OpenAPI format (contracts/api.yaml)
3. Generate quickstart guide with setup instructions and example usage
4. Implement configuration management with environment variable validation
5. Create Pydantic models for request/response validation
6. Implement FastAPI application with `/ask` endpoint
7. Wrap existing retrieval functionality in retrieval service
8. Define retrieve_documents tool for agent
9. Implement OpenAI agent with tool calling
10. Add error handling middleware with retry and circuit breaker
11. Implement comprehensive logging and monitoring

## Research Artifacts

- OpenAI Agents SDK documentation reviewed
- Cohere + Qdrant integration patterns validated
- Error handling strategies from production RAG systems
- FastAPI async patterns and best practices
- Performance benchmarks for RAG pipelines (typical: 3-5s response time)
