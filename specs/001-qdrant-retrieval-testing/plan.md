# Implementation Plan: Qdrant Retrieval Testing

**Branch**: `001-qdrant-retrieval-testing` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-qdrant-retrieval-testing/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of Qdrant retrieval testing functionality to verify that stored vectors can be accurately retrieved. This includes testing top-k matching, content integrity validation, metadata accuracy, and end-to-end pipeline functionality from input query to clean JSON output.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: cohere, qdrant-client, requests, beautifulsoup4, python-dotenv
**Storage**: Qdrant Cloud for vector storage
**Testing**: pytest for unit and integration tests
**Target Platform**: Linux server, Windows, macOS (cross-platform Python application)
**Project Type**: Backend service (single-file Python application)
**Performance Goals**: Sub-500ms response time for 95% of queries with top-k=5
**Constraints**: <500ms p95 response time, handle up to 2048 token chunks, maintain 99% uptime
**Scale/Scope**: Support up to 10,000 documents in collection, handle 100 concurrent queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Spec-Driven Development**: ✅ Aligned - following spec-driven approach with detailed requirements
- **Modular Architecture**: ⚠️ Single file implementation but with modular functions
- **AI Integration First**: ✅ Aligned - leveraging vector embeddings for retrieval
- **Performance & Scalability**: ✅ Aligned - designed for efficient retrieval with Qdrant
- **Security**: ✅ Aligned - proper API key handling through environment variables

## Project Structure

### Documentation (this feature)

```text
specs/001-qdrant-retrieval-testing/
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
├── main.py                 # Single file implementation with all functionality
├── pyproject.toml          # Project configuration with dependencies
├── uv.lock                 # UV package lock file
└── .env.example            # Environment variables template
```

**Structure Decision**: Single project structure chosen to match the requirement for a single-file implementation while maintaining modularity through well-defined functions. The implementation follows the RAG system approach outlined in the constitution.

## Phase 0: Research & Unknowns Resolution

### Research Tasks

#### R001: Determine test corpus for Qdrant retrieval validation
- **Objective**: Identify 3-5 documents from the target Docusaurus site to use as known corpus for testing
- **Approach**: Use documents from https://hassaanghayas.github.io/physical-ai-robotics-textbook/
- **Success Criteria**: Document URLs and expected content identified for baseline testing

#### R002: Define expected chunk texts, URLs, and chunk IDs for validation
- **Objective**: Create baseline data for testing chunk integrity and metadata accuracy
- **Approach**: Extract sample chunks from test corpus with their original content and metadata
- **Success Criteria**: Sample data with expected values documented for comparison

#### R003: Prepare test queries tied to corpus content
- **Objective**: Create specific queries that should match known corpus content
- **Approach**: Identify key concepts in corpus and create related queries that should return specific chunks
- **Success Criteria**: At least 5 test queries with expected results documented

### Dependencies
- Access to target Docusaurus site for test corpus
- Existing URL extraction functionality from previous implementation
- Working Qdrant connection with proper credentials

## Phase 1: Data Modeling & Contracts

### Data Models

#### SearchQuery Entity
- **Fields**:
  - query_text: string (the search query text)
  - top_k: int (number of results to return, default: 5)
  - filters: dict (optional filters for search, default: {})
- **Validation**: query_text must not be empty, top_k must be positive integer
- **Relationships**: None (request object)

#### RetrievalResult Entity
- **Fields**:
  - id: string (Qdrant point ID)
  - content: string (the retrieved text content)
  - url: string (source URL of the content)
  - chunk_id: string (ID of the chunk in the original document)
  - document_id: string (ID of the original document)
  - similarity_score: float (cosine similarity to query, range 0.0-1.0)
  - position: int (rank position in results, starting from 1)
- **Validation**: similarity_score must be between 0.0 and 1.0, position must be positive
- **Relationships**: References original Document and TextChunk

#### Metadata Entity
- **Fields**:
  - url: string (source document URL)
  - title: string (document title)
  - created_at: string (ISO timestamp)
  - document_id: string (original document ID)
  - chunk_index: int (position of chunk in document)
- **Validation**: url must be valid URL format, created_at must be valid ISO timestamp
- **Relationships**: Belongs to TextChunk

### API Contracts

#### Retrieval Endpoint
- **Method**: POST
- **Path**: /retrieve
- **Request Body**:
```json
{
  "query": "text query for vector search",
  "top_k": 5,
  "filters": {}
}
```
- **Response**:
```json
{
  "results": [
    {
      "id": "qdrant-point-id",
      "content": "retrieved text content",
      "url": "source-url",
      "chunk_id": "chunk-identifier",
      "document_id": "document-identifier",
      "similarity_score": 0.85,
      "position": 1
    }
  ]
}
```

#### Test Endpoint
- **Method**: POST
- **Path**: /test-retrieval
- **Request Body**:
```json
{
  "corpus_urls": ["url1", "url2"],
  "test_queries": ["query1", "query2"]
}
```
- **Response**:
```json
{
  "setup_complete": true,
  "expected_chunks": [...],
  "test_results": {
    "top_k_accuracy": 0.95,
    "chunk_integrity": 0.98,
    "metadata_correctness": 1.0
  }
}
```

## Phase 2: Implementation Approach

### Approach Summary
Implement a comprehensive test suite that validates the Qdrant retrieval functionality by ingesting known documents, recording expected values, and then testing retrieval accuracy. The implementation will follow the single-file approach with modular functions as required by the constitution.

### Component Breakdown

#### C001: Corpus Ingestion Module
- **Purpose**: Ingest a small, known corpus (3-5 docs) into Qdrant for testing
- **Interface**: Function to load documents from URLs and store with metadata
- **Dependencies**: Text extraction module, Qdrant client
- **Location**: backend/main.py

#### C002: Expected Value Recorder
- **Purpose**: Record expected chunk texts, URLs, and chunk_ids for validation
- **Interface**: Function to store baseline data for comparison during testing
- **Dependencies**: Corpus ingestion module
- **Location**: backend/main.py

#### C003: Query Preparation Module
- **Purpose**: Prepare test queries tied to the corpus content
- **Interface**: Function to generate queries based on corpus content that should return specific results
- **Dependencies**: Corpus ingestion module
- **Location**: backend/main.py

#### C004: Top-K Validation Module
- **Purpose**: Query Qdrant and confirm k results with valid IDs and expected ordering
- **Interface**: Function to run queries and validate result count and ordering
- **Dependencies**: Qdrant client
- **Location**: backend/main.py

#### C005: Chunk Integrity Validator
- **Purpose**: Verify returned text matches original chunks exactly or semantically
- **Interface**: Function to compare retrieved vs expected content with fuzzy matching
- **Dependencies**: Expected value recorder, top-k validator
- **Location**: backend/main.py

#### C006: Metadata Validator
- **Purpose**: Validate URL and chunk_id returned correctly with each result
- **Interface**: Function to compare retrieved vs expected metadata fields
- **Dependencies**: Expected value recorder, top-k validator
- **Location**: backend/main.py

#### C007: End-to-End Pipeline Tester
- **Purpose**: Run queries through full pipeline and verify clean JSON output
- **Interface**: Function to execute complete retrieval workflow and validate output format
- **Dependencies**: All above components
- **Location**: backend/main.py

### Implementation Sequence
1. Implement corpus ingestion (C001) - Set up test corpus in Qdrant
2. Implement expected value recorder (C002) - Record baseline data
3. Implement query preparation (C003) - Prepare test queries
4. Implement top-k validation (C004) - Validate result count and ordering
5. Implement chunk integrity validator (C005) - Validate content accuracy
6. Implement metadata validator (C006) - Validate metadata accuracy
7. Implement end-to-end pipeline tester (C007) - Complete pipeline validation

### Integration Points
- Integration with existing URL extraction functionality
- Integration with existing Qdrant storage mechanism
- Integration with existing embedding generation (using mock for testing)

## Phase 3: Deployment & Validation

### Test Environment Setup
- Configure test Qdrant collection for retrieval testing
- Set up test corpus documents
- Prepare validation dataset

### Validation Criteria
- Top-k results accuracy: 95% of queries return expected number of results
- Chunk integrity: 98% of retrieved chunks match original text or are semantically equivalent
- Metadata correctness: 100% of metadata fields (URL, chunk_id) correct
- Response time: 95% of queries complete within 500ms
- JSON output: 100% of responses follow expected schema

### Success Metrics
- All test queries return expected results
- Error rate below 1%
- Performance targets met
- End-to-end pipeline produces clean JSON output

## Risk Assessment

### Technical Risks
- **Qdrant connectivity**: Mitigation - implement retry logic and graceful degradation
- **API rate limits**: Mitigation - implement rate limiting and batch processing
- **Large document processing**: Mitigation - implement chunking and batching

### Schedule Risks
- **Dependency issues**: Mitigation - resolve early in Phase 0
- **Integration complexity**: Mitigation - implement modular components with clear interfaces

### Quality Risks
- **Incomplete test coverage**: Mitigation - define comprehensive test cases upfront
- **Performance issues**: Mitigation - implement with performance in mind from start