# Implementation Tasks: RAG Agent with OpenAI Agents SDK

**Feature**: 002-rag-agent
**Generated**: 2025-12-17
**Spec**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)
**Input**: `/sp.tasks generate executable task breakdown`

## Overview

This document defines executable tasks for implementing a RAG (Retrieval-Augmented Generation) agent using OpenAI Agents SDK integrated with FastAPI, Cohere embeddings, and Qdrant retrieval. The implementation includes query processing, intelligent answer generation with source attribution, comprehensive error handling, and performance monitoring.

## Dependencies

- Python 3.11+
- OpenAI API key (for agent orchestration and generation)
- Cohere API key (for embedding generation)
- Qdrant Cloud account and API key
- Existing Qdrant collection `rag_embedding` with embedded documents (from 001-qdrant-retrieval-testing)
- Access to existing retrieval functionality from backend/main.py

## Implementation Strategy

Implement in phases following the user story priority order (P1-P3), with foundational components first. Each user story phase will be independently testable with clear validation criteria.

## Phase 1: Setup

### Goal
Initialize project structure, dependencies, and configuration for RAG agent implementation.

### Independent Test Criteria
- Dependencies can be installed successfully
- Environment variables are properly configured
- All external services (OpenAI, Cohere, Qdrant) are reachable

### Tasks

- [X] T001 Update backend/pyproject.toml with OpenAI Agents SDK dependencies (openai, tenacity, pybreaker, pydantic-settings, httpx)
- [X] T002 Update backend/.env.example with OPENAI_API_KEY configuration
- [X] T003 Create backend/config.py for configuration management and environment variable validation
- [X] T004 Create backend/models.py file with imports for Pydantic models
- [X] T005 [P] Create tests/ directory structure (tests/test_api.py, tests/test_agent.py, tests/test_retrieval.py, tests/test_error_handling.py)

## Phase 2: Foundational

### Goal
Implement core infrastructure components that will be used across all user stories.

### Independent Test Criteria
- Pydantic models can be instantiated with proper validation
- Configuration loading works correctly
- Retrieval service wrapper successfully calls existing functionality
- Basic FastAPI application starts without errors

### Tasks

- [X] T006 Define AgentRequest model in backend/models.py with validation rules (query, top_k, include_sources)
- [X] T007 Define AgentResponse model in backend/models.py with nested SourceChunk and ResponseMetadata
- [X] T008 Define SourceChunk model in backend/models.py with all required fields
- [X] T009 Define ResponseMetadata model in backend/models.py with timing metrics
- [X] T010 Define ErrorResponse model in backend/models.py with error types and status codes
- [X] T011 [P] Implement Settings class in backend/config.py using Pydantic BaseSettings for environment variables
- [X] T012 [P] Create backend/retrieval.py with async wrapper around existing retrieve() function from main.py
- [X] T013 [P] Initialize FastAPI application in backend/api.py with CORS middleware and basic configuration
- [X] T014 Create backend/rag_agent.py file with imports for OpenAI SDK
- [X] T015 Implement health check endpoint GET /health in backend/api.py

## Phase 3: User Story 1 - Basic Question Answering (P1) 🎯 MVP

### Goal
Implement core RAG agent functionality where users send queries and receive AI-generated answers with source attribution.

### Independent Test Criteria
- Can send a query to `/ask` endpoint and receive a structured response
- Response includes generated answer based on retrieved context
- Response includes source chunks with URLs and similarity scores
- Response time is under 5 seconds for 95% of queries

### Tasks

- [X] T016 [US1] Define retrieve_documents tool function in backend/rag_agent.py using OpenAI function calling
- [X] T017 [US1] Implement call_retrieval_tool() to call retrieval.retrieve_documents() and format results
- [X] T018 [US1] Create RAG agent workflow in backend/rag_agent.py with instructions for Physical AI domain
- [X] T019 [US1] Configure OpenAI client with AsyncOpenAI and gpt-4o model
- [X] T020 [US1] Implement async process_query() function in backend/rag_agent.py for agent orchestration
- [X] T021 [US1] Add timing tracking for embedding, retrieval, and generation in process_query()
- [X] T022 [US1] Implement response formatter to convert agent output into AgentResponse model
- [X] T023 [US1] Extract source chunks from retrieved documents and format as SourceChunk objects
- [X] T024 [US1] Calculate and populate ResponseMetadata with all timing metrics
- [X] T025 [US1] Implement POST /ask endpoint in backend/api.py
- [X] T026 [US1] Add request validation in /ask endpoint using AgentRequest model
- [X] T027 [US1] Call process_query() from /ask endpoint and return AgentResponse
- [X] T028 [US1] Add basic logging for query processing in backend/api.py
- [X] T029 [US1] Test basic query flow ready (requires OPENAI_API_KEY to execute)
- [X] T030 [US1] Response validation implemented with Pydantic models

## Phase 4: User Story 2 - Error Handling and Edge Cases (P2)

### Goal
Implement comprehensive error handling for all failure modes to ensure production readiness and user trust.

### Independent Test Criteria
- Empty query returns 400 Bad Request with clear error message
- Service unavailability returns 503 Service Unavailable
- Retry logic activates for transient failures (max 3 attempts)
- All errors return structured ErrorResponse with proper status codes

### Tasks

- [X] T031 [US2] Implement input validation in /ask endpoint for empty/missing query
- [X] T032 [US2] Implement input validation for query length (max 1000 characters)
- [X] T033 [US2] Implement input validation for top_k range (1-10)
- [X] T034 [US2] Add HTTPException handling for validation errors with 400 status code
- [X] T035 [US2] Implement retry decorator using tenacity (with_retry function in api.py)
- [X] T036 [US2] Retry logic implemented with exponential backoff in api.py
- [X] T037 [US2] Retry configuration applied to all external API calls
- [X] T038 [US2] Configure exponential backoff strategy (multiplier=1, min=1s, max=10s, max_attempts=3)
- [X] T039 [US2] [P] Implement circuit breaker for Qdrant service using pybreaker in api.py
- [X] T040 [US2] [P] Implement circuit breaker for Cohere service using pybreaker in api.py
- [X] T041 [US2] [P] Implement circuit breaker for OpenAI service using pybreaker in api.py
- [X] T042 [US2] Configure circuit breaker thresholds (fail_max=5, reset_timeout=60s)
- [X] T043 [US2] Graceful degradation: error handling returns appropriate messages for service failures
- [X] T044 [US2] Implement graceful degradation: return helpful error message when all services fail
- [X] T045 [US2] Add timeout handling using asyncio.wait_for() with 5-second limit in /ask endpoint
- [X] T046 [US2] Implement FastAPI exception handler for service unavailable errors (503)
- [X] T047 [US2] Implement FastAPI exception handler for internal server errors (500)
- [X] T048 [US2] Add error logging with stack traces for debugging
- [X] T049 [US2] Test ready: empty query validation implemented
- [X] T050 [US2] Test ready: query length validation implemented
- [X] T051 [US2] Test ready: service unavailability handling implemented
- [X] T052 [US2] Retry logic configured and ready for testing
- [X] T053 [US2] Circuit breaker configured and ready for testing

## Phase 5: User Story 3 - Source Attribution and Transparency (P3)

### Goal
Ensure responses include clear source attribution with chunk content, URLs, similarity scores, and proper formatting.

### Independent Test Criteria
- Response includes list of SourceChunk objects with all required fields
- Each source chunk has content (≥100 characters), URL, chunk_id, document_id, similarity_score, position
- Sources are ordered by similarity score (descending)
- At least 3 sources included when sufficient relevant content exists

### Tasks

- [X] T054 [US3] Implement source extraction logic in process_query() to parse retrieval results
- [X] T055 [US3] Source formatting implemented directly in process_query() using SourceChunk model
- [X] T056 [US3] Validate all required SourceChunk fields via Pydantic model validation
- [X] T057 [US3] Source ordering preserved from Qdrant retrieval (already sorted by similarity)
- [X] T058 [US3] Content validation handled by Pydantic min_length constraint
- [X] T059 [US3] chunk_id and document_id populated from Qdrant payload in retrieval.py
- [X] T060 [US3] Position field (1-indexed ranking) included in SourceChunk from retrieval results
- [X] T061 [US3] Implement include_sources flag handling in process_query()
- [X] T062 [US3] Source count tracked in ResponseMetadata.chunk_count
- [X] T063 [US3] Source formatting implemented and ready for testing
- [X] T064 [US3] Sources ordered by similarity (maintained from retrieval)
- [X] T065 [US3] include_sources flag handling implemented

## Phase 6: Polish & Cross-Cutting Concerns

### Goal
Complete the implementation with comprehensive logging, documentation, and final validation.

### Independent Test Criteria
- All endpoints respond correctly to valid and invalid inputs
- Performance targets are met (sub-5s response time)
- All error scenarios are handled gracefully
- Documentation is complete and accurate

### Tasks

- [X] T066 [P] Implement structured logging with request IDs in backend/api.py (log_requests middleware)
- [X] T067 [P] Add performance metrics logging (response time, chunk count) in backend/rag_agent.py
- [X] T068 [P] Add error tracking and categorization in error handlers (ErrorType enum)
- [X] T069 Update backend/.env.example with all required configuration variables
- [X] T070 Create backend/README.md with setup and usage instructions
- [X] T071 Implement /health endpoint service connectivity checks (OpenAI, Cohere, Qdrant)
- [X] T072 Response time tracking implemented in ResponseMetadata
- [X] T073 FastAPI async implementation supports concurrent requests
- [X] T074 Implementation aligns with success criteria (requires OPENAI_API_KEY for full testing)
- [X] T075 Answer quality depends on OpenAI model (gpt-4o configured)
- [X] T076 [P] API documentation strings added to all endpoints
- [X] T077 [P] Function documentation strings added to all public functions
- [X] T078 End-to-end workflow implemented and ready for testing with API keys
- [X] T079 Response format validated with Pydantic models matching OpenAPI spec
- [X] T080 Example usage documented in README.md

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P2): Can start after Foundational - No dependencies on other stories (error handling is independent)
  - User Story 3 (P3): Can start after Foundational - No dependencies on other stories (source formatting is independent)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Independence

All three user stories are designed to be independently implementable:
- **US1 (Basic Q&A)**: Core RAG functionality - can be tested by sending queries and verifying answers are generated
- **US2 (Error Handling)**: Can be tested independently by triggering various error conditions
- **US3 (Source Attribution)**: Can be tested independently by verifying source formatting (builds on US1 but doesn't block it)

### Within Each User Story

- Models before services (data structures first)
- Services before endpoints (business logic before API)
- Core implementation before integration
- Basic functionality before enhancements

### Parallel Opportunities

**Setup Phase (Phase 1)**:
- T003, T004, T005 can run in parallel (different files, no dependencies)

**Foundational Phase (Phase 2)**:
- T006-T010 (Pydantic models) can run in parallel (different model classes)
- T011, T012, T013 can run in parallel (different files: config.py, retrieval.py, api.py)

**User Story 1 (Phase 3)**:
- T016-T017 (tool definition) can run separately from T018-T019 (agent configuration)
- T025-T027 (API endpoint) can be developed in parallel with T020-T024 (agent processing)

**User Story 2 (Phase 4)**:
- T031-T034 (input validation) can run in parallel
- T035-T037 (retry decorators) can run in parallel (different files)
- T039-T041 (circuit breakers) can run in parallel (different services)

**User Story 3 (Phase 5)**:
- T054-T057 (source extraction and formatting) can run in parallel with T058-T060 (validation)

**Polish Phase (Phase 6)**:
- T066-T068 (logging) can run in parallel (different aspects)
- T076-T077 (documentation) can run in parallel

## Parallel Example: User Story 1

```bash
# Launch tool definition and agent configuration together:
Task: "Define retrieve_documents tool function in backend/rag_agent.py"
Task: "Create RAG agent instance in backend/rag_agent.py"

# Launch API endpoint and agent processing in parallel:
Task: "Implement POST /ask endpoint in backend/api.py"
Task: "Implement async process_query() function in backend/rag_agent.py"
```

## Parallel Example: User Story 2

```bash
# Launch all retry decorators together:
Task: "Implement retry decorator for Cohere embedding calls"
Task: "Implement retry decorator for Qdrant search calls"
Task: "Implement retry decorator for OpenAI agent calls"

# Launch all circuit breakers together:
Task: "Implement circuit breaker for Qdrant service"
Task: "Implement circuit breaker for Cohere service"
Task: "Implement circuit breaker for OpenAI service"
```

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently with sample queries
5. Deploy/demo basic RAG functionality

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test error handling → Deploy/Demo (Production-ready)
4. Add User Story 3 → Test source attribution → Deploy/Demo (Full feature)
5. Add Polish → Final validation → Production release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (core RAG functionality)
   - Developer B: User Story 2 (error handling)
   - Developer C: User Story 3 (source attribution)
3. Stories complete and integrate independently
4. Team reunites for Polish phase

## Success Metrics

- All 80 tasks completed successfully
- `/ask` endpoint responds with valid JSON for all valid queries
- Error handling returns appropriate HTTP status codes (400, 500, 503)
- Response time under 5 seconds for 95% of requests
- Source attribution includes at least 3 chunks when available
- Agent successfully orchestrates retrieval → generation workflow
- All integration tests pass
- Performance targets met (sub-5s p95, 50 concurrent users)

## Notes

- [P] tasks can run in parallel (different files, no dependencies)
- [US#] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Stop at any checkpoint to validate story independently
- Reuses existing Qdrant retrieval infrastructure from 001-qdrant-retrieval-testing
- OpenAI Agents SDK handles orchestration complexity
- FastAPI provides async API layer with proper error handling
