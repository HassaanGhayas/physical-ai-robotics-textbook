# Implementation Plan: Embedding Pipeline Setup

**Branch**: `001-embedding-pipeline` | **Date**: 2025-12-11 | **Spec**: [specs/001-embedding-pipeline/spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-embedding-pipeline/spec.md` and implementation requirements from `prompt.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The Embedding Pipeline Setup implements a backend system that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval. The system will be implemented as a single Python file (`main.py`) with functionality to crawl URLs, clean and chunk text, generate embeddings, and store them in Qdrant with metadata.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- Cohere client library for embedding generation
- Qdrant client library for vector storage
- Requests/BeautifulSoup for web crawling and text extraction
- UV package manager for project initialization
- FastAPI (as per constitution requirements) for potential API endpoints
**Storage**: Qdrant vector database for embedding storage
**Testing**: Unit tests for core functions (URL crawling, text extraction, embedding generation)
**Target Platform**: Backend service (standalone Python application)
**Performance Goals**: Efficient processing of Docusaurus URLs with reasonable embedding generation time, support for batch processing
**Constraints**: Single file implementation (`main.py`), integration with Cohere and Qdrant APIs, text cleaning and chunking requirements
**Scale/Scope**: Initial implementation for Physical AI & Humanoid Robotics textbook content with potential for expansion to other Docusaurus sites

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the Physical AI & Humanoid Robotics Book Constitution:

1. **Spec-Driven Development**: ✅ Following Spec-Kit Plus methodology with Claude Code and Spec-Kit Plus
2. **Modular Architecture**: ⚠️ Single file implementation (main.py) but following modular function design with separate concerns
3. **AI Integration First**: ✅ Using Cohere Models for embedding generation as specified in constitution
4. **Authentication & Personalization**: N/A - Backend service for RAG pipeline
5. **Performance & Scalability**: ✅ Using Qdrant Cloud for scalable vector search as specified in constitution
6. **Multi-language Support**: N/A - Backend service for RAG pipeline

The embedding pipeline aligns with the constitution's requirements for AI integration (Cohere Models) and scalable vector storage (Qdrant Cloud). The implementation follows the RAG system approach outlined in the constitution.

## Project Structure

### Documentation (this feature)

```text
specs/001-embedding-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (new backend folder)

```text
backend/
├── main.py                 # Single file implementation with all functionality
├── pyproject.toml          # Project configuration with dependencies
├── uv.lock                 # UV package lock file
├── requirements.txt        # Dependencies (if needed)
└── .env.example            # Environment variables template
```

### Key Components in main.py

```text
main.py contains:
├── get_all_urls()          # Function to extract all URLs from the Docusaurus site
├── extract_text_from_url() # Function to extract clean text from a URL
├── chunk_text()            # Function to chunk text into manageable pieces
├── create_collection()     # Function to create Qdrant collection named 'rag_embedding'
├── save_chunk_to_qdrant()  # Function to save chunked text with embeddings to Qdrant
├── main()                  # Main execution function orchestrating the pipeline
└── Cohere and Qdrant setup # Client initialization and configuration
```

**Structure Decision**: The backend is organized as a single Python file (`main.py`) following the requirement from the prompt. The implementation will have separate functions for each major component to maintain modularity despite the single-file constraint. The project will be initialized with UV package manager and include proper configuration files.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Single file implementation | Requirement specified in prompt to use only one file named main.py | Multi-file approach would be more maintainable but violates the explicit requirement |

## Phase 0: Research & Unknowns Resolution

### Research Tasks

1. **Cohere Integration Research**
   - Decision: Use Cohere's Python SDK for embedding generation
   - Rationale: Official SDK provides best performance and features
   - Alternatives considered: Direct API calls via requests library

2. **Qdrant Integration Research**
   - Decision: Use Qdrant Python client for vector database operations
   - Rationale: Official client provides best integration and features
   - Alternatives considered: Direct API calls via requests library

3. **Web Crawling Approach Research**
   - Decision: Use requests + BeautifulSoup for URL crawling and text extraction
   - Rationale: Proven approach for extracting content from static sites
   - Alternatives considered: Selenium for dynamic content, scrapy for complex crawling

4. **Text Chunking Strategy Research**
   - Decision: Use semantic chunking based on document structure
   - Rationale: Preserves context while keeping chunks manageable
   - Alternatives considered: Fixed-length character/word chunks

### Dependencies Analysis

- **cohere**: For embedding generation
- **qdrant-client**: For vector database operations
- **requests**: For web crawling
- **beautifulsoup4**: For HTML parsing
- **python-dotenv**: For environment variable management

## Phase 1: Data Model & Contracts

### Key Entities

**Document**
- url: string (source URL)
- title: string (page title)
- content: string (cleaned text content)
- created_at: timestamp

**Embedding**
- document_id: string (reference to source document)
- content: string (chunked content)
- embedding_vector: list[float] (vector representation)
- metadata: dict (additional information)
- created_at: timestamp

**Retrieval Result**
- document_id: string (reference to source document)
- content: string (retrieved content)
- similarity_score: float (relevance score)
- metadata: dict (document metadata)

### API Contract (if needed)

Since this is primarily a data pipeline, the main interface will be the functions in main.py, but if API functionality is needed later:

```
POST /process-url
- Request: {url: string, chunk_size: int}
- Response: {status: string, chunks_processed: int, collection_name: string}

GET /search
- Request: {query: string, top_k: int}
- Response: {results: [RetrievalResult]}
```

### Quickstart Guide

1. Install dependencies: `uv sync`
2. Set environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY)
3. Run the pipeline: `python main.py`
4. The pipeline will:
   - Crawl the specified Docusaurus site
   - Extract and clean text content
   - Generate embeddings using Cohere
   - Store embeddings in Qdrant collection named 'rag_embedding'

## Implementation Approach

The implementation will follow these steps in main.py:
1. Initialize Cohere and Qdrant clients
2. Define functions for each component (URL extraction, text extraction, chunking, etc.)
3. Create Qdrant collection named 'rag_embedding'
4. Implement the main execution flow
5. Handle errors and logging appropriately

The target deployment URL is: hassaanghayas.github.io/physical-ai-robotics-textbook/
**SiteMap URL**:https://hassaanghayas.github.io/physical-ai-robotics-textbook/sitemap.xml