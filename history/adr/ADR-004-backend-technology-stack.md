# ADR-004: Backend Technology Stack for Embedding Pipeline

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** Embedding Pipeline Setup
- **Context:** Need to select a technology stack for the backend service that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval. The solution must be efficient, maintainable, and integrate well with AI/ML services.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Language: Python 3.11+ (for AI/ML ecosystem and text processing capabilities)
- Embedding Service: Cohere Python SDK (for embedding generation)
- Vector Database: Qdrant Python client (for vector storage and retrieval)
- Web Crawling: requests + BeautifulSoup4 (for content extraction)
- Package Management: UV with pyproject.toml (for fast dependency management)
- Potential API: FastAPI (for future RAG interface as per constitution)

## Consequences

### Positive

- Rich ecosystem for AI/ML and text processing
- Official SDKs provide best integration with Cohere and Qdrant
- Lightweight crawling solution appropriate for static sites
- Fast dependency resolution with UV
- FastAPI aligns with constitution requirements for API services
- Python's data science ecosystem supports future enhancements

### Negative

- Additional dependencies increase deployment complexity
- Python may have performance limitations compared to compiled languages
- Rate limits on external APIs (Cohere, web requests) require careful management
- Memory usage for embedding generation can be significant
- Dependency on external API services for core functionality

## Alternatives Considered

Alternative Stack A: Node.js + TypeScript + Pinecone
- Why rejected: Less mature AI/ML ecosystem compared to Python, different skill set required

Alternative Stack B: Go + custom HTTP client + Weaviate
- Why rejected: Less AI/ML ecosystem support, more complex for text processing tasks

Alternative Stack C: Java + Spring Boot + Elasticsearch dense vector support
- Why rejected: Heavier framework, less suitable for AI/ML tasks, more verbose code

## References

- Feature Spec: specs/001-embedding-pipeline/spec.md
- Implementation Plan: specs/001-embedding-pipeline/plan.md
- Related ADRs: ADR-001-technology-stack-and-architecture.md (complements frontend approach)
- Evaluator Evidence: specs/001-embedding-pipeline/research.md