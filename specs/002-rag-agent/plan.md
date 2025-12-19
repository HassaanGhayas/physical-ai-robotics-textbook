# Implementation Plan: RAG Agent with OpenAI Agents SDK

**Branch**: `002-rag-agent` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-rag-agent/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of an intelligent RAG (Retrieval-Augmented Generation) agent using OpenAI Agents SDK that accepts natural language queries, retrieves relevant information from a vector database (Qdrant), and generates contextually accurate answers with source attribution. The system exposes a FastAPI `/ask` endpoint, integrates Cohere for embeddings and Qdrant for retrieval, and uses OpenAI's agent orchestration for sophisticated query handling.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: openai-agents-sdk, fastapi, uvicorn, cohere, qdrant-client, pydantic, python-dotenv, httpx
**Storage**: Qdrant Cloud (vector database) for document embeddings and retrieval
**Testing**: pytest, pytest-asyncio, httpx (for API testing)
**Target Platform**: Linux/Windows server (cross-platform Python application)
**Project Type**: Backend API service with agent orchestration
**Performance Goals**: Sub-5-second response time for 95% of queries, support up to 50 concurrent users
**Constraints**: <5s p95 response time, proper error handling for all failure modes, stateless query processing
**Scale/Scope**: Single API endpoint (`/ask`), integration with existing Qdrant collection (22 documents), support for 50 concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Spec-Driven Development**: ✅ Aligned - Following Spec-Kit Plus methodology with clear spec and planning phases
- **Modular Architecture**: ✅ Aligned - Separate concerns: FastAPI for API, OpenAI Agents SDK for orchestration, Cohere for embeddings, Qdrant for retrieval
- **AI Integration First**: ✅ Aligned - RAG agent is the core feature, using OpenAI Agents SDK, Cohere Models, and Qdrant Cloud as specified in constitution
- **Performance & Scalability**: ✅ Aligned - Designed for sub-5-second response with support for 50 concurrent users, leveraging existing Qdrant Cloud infrastructure
- **Authentication & Personalization**: ⚠️ Out of scope for v1 - Authentication explicitly excluded per spec (all requests treated equally). Will be added in future iteration aligned with Better Auth constitution requirement.

## Project Structure

### Documentation (this feature)

```text
specs/002-rag-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── rag_agent.py         # Main RAG agent implementation with OpenAI Agents SDK
├── api.py               # FastAPI application with /ask endpoint
├── models.py            # Pydantic models for request/response validation
├── config.py            # Configuration management and environment variables
├── retrieval.py         # Integration with existing Qdrant retrieval (from 001)
├── pyproject.toml       # Project dependencies and configuration
├── uv.lock              # UV package lock file
└── .env.example         # Environment variables template

tests/
├── test_api.py          # API endpoint tests
├── test_agent.py        # Agent orchestration tests
├── test_retrieval.py    # Retrieval integration tests
└── test_error_handling.py # Error handling and edge case tests
```

**Structure Decision**: Backend-only structure chosen to match the requirement for a REST API service. The implementation will be modular with separate files for agent logic, API layer, models, configuration, and retrieval integration. This aligns with the modular architecture principle while keeping the codebase focused and maintainable.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Authentication deferred | v1 focuses on core RAG functionality | Adding Better Auth now would delay MVP delivery; authentication can be layered on top of stateless API in v2 without architectural changes |

## Phase 0: Research & Unknowns Resolution

### Research Tasks

#### R001: OpenAI Agents SDK Architecture and Best Practices
- **Objective**: Understand how to structure agents using OpenAI Agents SDK, including tool definitions, orchestration patterns, and integration with external services
- **Approach**: Research official documentation, example implementations, and best practices for RAG use cases
- **Success Criteria**: Clear understanding of agent structure, tool calling patterns, and error handling strategies
-

 **Expected Outcome**: Architecture diagram showing agent flow and component interactions

#### R002: Integration Pattern for Cohere + Qdrant + OpenAI
- **Objective**: Define the optimal integration pattern for combining Cohere embeddings, Qdrant retrieval, and OpenAI generation in an agent workflow
- **Approach**: Research existing integration patterns, evaluate tradeoffs between different orchestration approaches
- **Success Criteria**: Documented integration pattern with clear data flow and error handling
- **Expected Outcome**: Sequence diagram showing the complete RAG pipeline from query to response

#### R003: FastAPI + Async Agent Pattern
- **Objective**: Determine best practices for exposing agent functionality via FastAPI, including async handling, timeouts, and streaming responses
- **Approach**: Research FastAPI async patterns, agent invocation strategies, and timeout handling
- **Success Criteria**: Clear pattern for API → Agent integration with proper error boundaries
- **Expected Outcome**: Code structure pattern for FastAPI endpoints calling async agents

#### R004: Error Handling and Retry Logic for Multi-Service Architecture
- **Objective**: Define comprehensive error handling strategy for coordinating failures across Cohere, Qdrant, and OpenAI APIs
- **Approach**: Research circuit breaker patterns, retry strategies, and graceful degradation approaches
- **Success Criteria**: Documented error handling matrix showing failure modes and recovery strategies
- **Expected Outcome**: Error handling guidelines and retry configuration recommendations

#### R005: Tool Definition for Retrieval Integration
- **Objective**: Define how to expose Qdrant retrieval as a tool within the OpenAI Agents SDK framework
- **Approach**: Research tool definition patterns, parameter passing, and result formatting
- **Success Criteria**: Clear tool schema that the agent can call for retrieval operations
- **Expected Outcome**: Tool definition specification compatible with OpenAI Agents SDK

### Dependencies

- Access to OpenAI Agents SDK documentation and examples
- Existing Qdrant retrieval implementation from 001-qdrant-retrieval-testing
- Cohere API access for embedding generation
- OpenAI API access for agent orchestration and generation
- FastAPI framework documentation

## Phase 1: Data Modeling & Contracts

### Data Models

#### AgentRequest Entity
- **Fields**:
  - query: string (required, max 1000 characters, the user's natural language question)
  - top_k: integer (optional, default: 5, number of chunks to retrieve, range: 1-10)
  - include_sources: boolean (optional, default: true, whether to include source attribution)
- **Validation**: Query must not be empty after trimming, top_k must be positive integer
- **Relationships**: None (request object)

#### AgentResponse Entity
- **Fields**:
  - answer: string (the generated natural language answer from the agent)
  - sources: list[SourceChunk] (retrieved document chunks used to generate the answer)
  - metadata: ResponseMetadata (response timing and status information)
- **Validation**: Answer must not be empty, sources list should have at least 1 item when available
- **Relationships**: Contains multiple SourceChunk entities

#### SourceChunk Entity
- **Fields**:
  - content: string (the text content of the retrieved chunk)
  - url: string (source URL of the original document)
  - chunk_id: string (identifier for the specific chunk)
  - document_id: string (identifier for the parent document)
  - similarity_score: float (cosine similarity score, range: 0.0-1.0)
  - position: integer (rank in retrieval results, 1-indexed)
- **Validation**: Similarity score must be between 0.0 and 1.0, position must be positive
- **Relationships**: Part of AgentResponse, references original documents in Qdrant

#### ResponseMetadata Entity
- **Fields**:
  - response_time_ms: float (total time taken to process the request in milliseconds)
  - chunk_count: integer (number of retrieved chunks)
  - embedding_time_ms: float (time taken for embedding generation)
  - retrieval_time_ms: float (time taken for Qdrant retrieval)
  - generation_time_ms: float (time taken for OpenAI generation)
- **Validation**: All time fields must be non-negative
- **Relationships**: Part of AgentResponse

#### ErrorResponse Entity
- **Fields**:
  - error: string (error type or category)
  - message: string (human-readable error message)
  - detail: string (optional, additional debugging information)
  - status_code: integer (HTTP status code)
- **Validation**: Status code must be valid HTTP code (400-599)
- **Relationships**: None (error response object)

### API Contracts

#### POST /ask - Ask a Question to the RAG Agent
- **Method**: POST
- **Path**: /ask
- **Request Body**:
```json
{
  "query": "What is physical AI?",
  "top_k": 5,
  "include_sources": true
}
```
- **Success Response** (200 OK):
```json
{
  "answer": "Physical AI refers to artificial intelligence systems that interact with and manipulate the physical world through embodied agents such as robots. It combines AI algorithms with mechanical actuators and sensors to enable intelligent physical interactions.",
  "sources": [
    {
      "content": "Physical AI involves sophisticated AI algorithms controlling complex mechanical bodies...",
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
- **Error Responses**:
  - 400 Bad Request: Invalid query (empty, too long, malformed JSON)
  - 500 Internal Server Error: Agent processing failure
  - 503 Service Unavailable: External service (Cohere, Qdrant, OpenAI) unavailable

#### GET /health - Health Check Endpoint
- **Method**: GET
- **Path**: /health
- **Response** (200 OK):
```json
{
  "status": "healthy",
  "services": {
    "openai": "connected",
    "cohere": "connected",
    "qdrant": "connected"
  },
  "timestamp": "2025-12-17T12:00:00Z"
}
```

## Phase 2: Implementation Approach

### Approach Summary
Implement a RAG agent using OpenAI Agents SDK that orchestrates the retrieval and generation workflow. The agent will use Cohere for embeddings, Qdrant for retrieval (reusing existing infrastructure from 001-qdrant-retrieval-testing), and OpenAI for answer generation. FastAPI will expose the agent functionality via REST API with comprehensive error handling and monitoring.

### Component Breakdown

#### C001: FastAPI Application Setup
- **Purpose**: Initialize FastAPI application with proper middleware, error handlers, and CORS configuration
- **Interface**: FastAPI app instance with configured routes and middleware
- **Dependencies**: FastAPI, Uvicorn, python-dotenv
- **Location**: backend/api.py

#### C002: Pydantic Models
- **Purpose**: Define request/response models for API validation and serialization
- **Interface**: Pydantic BaseModel classes for all API contracts
- **Dependencies**: Pydantic v2
- **Location**: backend/models.py

#### C003: Configuration Management
- **Purpose**: Load and validate environment variables, API keys, and configuration settings
- **Interface**: Configuration class with environment variable validation
- **Dependencies**: python-dotenv, pydantic-settings
- **Location**: backend/config.py

#### C004: Retrieval Service Integration
- **Purpose**: Integrate with existing Qdrant retrieval from 001-qdrant-retrieval-testing
- **Interface**: retrieve(query: str, top_k: int) → List[RetrievalResult]
- **Dependencies**: Existing backend/main.py retrieve function, qdrant-client, cohere
- **Location**: backend/retrieval.py (wrapper around existing retrieve function)

#### C005: OpenAI Agent Definition
- **Purpose**: Define the RAG agent using OpenAI Agents SDK with retrieval tool
- **Interface**: RAGAgent class with query processing and tool calling capabilities
- **Dependencies**: openai-agents-sdk, openai
- **Location**: backend/rag_agent.py

#### C006: Retrieval Tool for Agent
- **Purpose**: Expose Qdrant retrieval as a tool that the agent can call
- **Interface**: Tool definition compatible with OpenAI Agents SDK
- **Dependencies**: OpenAI Agents SDK, retrieval service
- **Location**: backend/rag_agent.py

#### C007: Agent Orchestration
- **Purpose**: Orchestrate the complete RAG workflow: query → embed → retrieve → generate → respond
- **Interface**: async process_query(query: AgentRequest) → AgentResponse
- **Dependencies**: All above components
- **Location**: backend/rag_agent.py

#### C008: Error Handling Middleware
- **Purpose**: Catch and handle errors from external APIs (Cohere, Qdrant, OpenAI) with retry logic
- **Interface**: FastAPI exception handlers and retry decorators
- **Dependencies**: tenacity (for retries), FastAPI
- **Location**: backend/api.py

#### C009: Response Formatter
- **Purpose**: Format agent output into structured JSON response with source attribution
- **Interface**: format_response(agent_output, sources, metadata) → AgentResponse
- **Dependencies**: Pydantic models
- **Location**: backend/rag_agent.py

#### C010: Logging and Monitoring
- **Purpose**: Log all requests, responses, errors, and performance metrics
- **Interface**: Structured logging with request IDs and timing information
- **Dependencies**: Python logging, structlog (optional)
- **Location**: backend/api.py, backend/rag_agent.py

### Implementation Sequence
1. Implement Configuration Management (C003) - Load environment variables and API keys
2. Implement Pydantic Models (C002) - Define API contracts
3. Implement FastAPI Application Setup (C001) - Initialize FastAPI with basic structure
4. Implement Retrieval Service Integration (C004) - Wrap existing Qdrant retrieval
5. Implement Retrieval Tool for Agent (C006) - Define tool schema for OpenAI Agents SDK
6. Implement OpenAI Agent Definition (C005) - Create RAG agent with tool calling
7. Implement Agent Orchestration (C007) - Connect all components in workflow
8. Implement Response Formatter (C009) - Format outputs into API responses
9. Implement Error Handling Middleware (C008) - Add retry logic and error handlers
10. Implement Logging and Monitoring (C010) - Add comprehensive logging

### Integration Points
- Integration with existing Qdrant collection `rag_embedding` from 001-qdrant-retrieval-testing
- Integration with existing retrieval function from backend/main.py
- Integration with Cohere API for embedding generation (reuse existing setup)
- Integration with OpenAI API for agent orchestration and generation
- FastAPI integration with async agent execution

## Phase 3: Deployment & Validation

### Test Environment Setup
- Configure environment variables for OpenAI, Cohere, and Qdrant APIs
- Ensure Qdrant collection `rag_embedding` is accessible and populated
- Set up pytest environment with async support

### Validation Criteria
- `/ask` endpoint responds with valid JSON for all valid queries
- Error handling returns appropriate HTTP status codes (400, 500, 503)
- Response time under 5 seconds for 95% of requests
- Source attribution includes at least 3 chunks when available
- Agent successfully orchestrates retrieval → generation workflow
- Retry logic handles transient failures from external APIs

### Success Metrics
- All unit tests pass (targeting 90%+ coverage)
- Integration tests pass for happy path and error scenarios
- Load testing shows support for 50 concurrent users
- Average response time under 3 seconds
- Zero unhandled exceptions in production testing

## Risk Assessment

### Technical Risks
- **OpenAI Agents SDK complexity**: Mitigation - Start with simple agent implementation, add sophistication incrementally
- **Rate limiting on APIs**: Mitigation - Implement retry logic with exponential backoff, consider caching (future)
- **Response time variability**: Mitigation - Set aggressive timeouts, implement async processing, monitor p95 latency

### Schedule Risks
- **OpenAI Agents SDK learning curve**: Mitigation - Phase 0 research dedicated to understanding SDK patterns
- **Integration complexity**: Mitigation - Reuse existing retrieval infrastructure, test components independently

### Quality Risks
- **Error handling gaps**: Mitigation - Comprehensive error matrix, test all failure modes
- **Agent output quality**: Mitigation - Test with diverse queries, validate source attribution accuracy
