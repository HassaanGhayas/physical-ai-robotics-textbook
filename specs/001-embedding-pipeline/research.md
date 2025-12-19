# Research: Embedding Pipeline Implementation

**Feature**: Embedding Pipeline Setup
**Date**: 2025-12-11
**Branch**: 001-embedding-pipeline

## Research Summary

This research document covers the technical decisions and approaches for implementing the embedding pipeline that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Cohere Integration Research

### Decision: Use Cohere's Python SDK for embedding generation

**Rationale**: The official Cohere Python SDK provides the most reliable and feature-rich interface to the Cohere API. It includes built-in error handling, rate limiting management, and support for all embedding models.

**Alternatives Considered**:
1. Direct API calls via requests library
   - Pros: More control over requests, lighter dependency
   - Cons: Manual error handling, rate limiting, and response parsing required

**Selected Approach**: Cohere Python SDK (cohere>=5.0.0)

### Implementation Details:
- Use `cohere.Client(api_key=API_KEY)` for client initialization
- Use `client.embed()` method for generating embeddings
- Support for different embedding models (e.g., 'embed-english-v3.0')

## Qdrant Integration Research

### Decision: Use Qdrant Python client for vector database operations

**Rationale**: The official Qdrant Python client provides the most comprehensive interface to Qdrant's features, including collection management, point operations, and search capabilities.

**Alternatives Considered**:
1. Direct API calls via requests library
   - Pros: More control over requests, lighter dependency
   - Cons: Manual handling of all Qdrant operations, error management

**Selected Approach**: Qdrant Client (qdrant-client>=1.9.0)

### Implementation Details:
- Use `QdrantClient()` for client initialization (supporting both cloud and local instances)
- Create collection named 'rag_embedding' with appropriate vector dimensions
- Use upsert operations for storing embeddings with metadata

## Web Crawling Approach Research

### Decision: Use requests + BeautifulSoup for URL crawling and text extraction

**Rationale**: This combination provides a lightweight, reliable solution for extracting content from static sites like Docusaurus. It's efficient for the use case and doesn't require heavy browser automation.

**Alternatives Considered**:
1. Selenium for dynamic content
   - Pros: Can handle JavaScript-rendered content
   - Cons: Heavy, slower, requires browser management
2. Scrapy for complex crawling
   - Pros: Powerful for complex crawling scenarios
   - Cons: Overkill for simple Docusaurus site crawling

**Selected Approach**: requests + BeautifulSoup4

### Implementation Details:
- Use `requests.get()` to fetch Docusaurus pages
- Use BeautifulSoup to parse HTML and extract text content
- Focus on main content areas while excluding navigation, headers, footers

## Text Chunking Strategy Research

### Decision: Use semantic chunking based on document structure

**Rationale**: Semantic chunking preserves the context and meaning of content while keeping chunks manageable for embedding generation. This approach respects document structure (headings, paragraphs) to maintain coherence.

**Alternatives Considered**:
1. Fixed-length character chunks
   - Pros: Simple implementation
   - Cons: May break context, poor semantic coherence
2. Fixed-length word chunks
   - Pros: Better than character chunks
   - Cons: Still may break context within sentences

**Selected Approach**: Semantic chunking with configurable size limits

### Implementation Details:
- Respect document structure (headings, paragraphs)
- Maximum chunk size: 512 tokens (configurable)
- Overlap between chunks: 50 tokens (to maintain context)
- Preserve headings and context when possible

## Dependency Management Research

### Decision: Use UV package manager with pyproject.toml

**Rationale**: UV is a fast Python package installer and resolver that works well with pyproject.toml. It provides fast dependency resolution and installation, which is important for the backend service.

**Selected Dependencies**:
- cohere: For embedding generation
- qdrant-client: For vector database operations
- requests: For web crawling
- beautifulsoup4: For HTML parsing
- python-dotenv: For environment variable management
- fastapi: For potential API functionality (as per constitution)
- uvicorn: For running FastAPI applications

## Error Handling Strategy Research

### Decision: Implement comprehensive error handling with retry logic

**Rationale**: The pipeline involves multiple external services (web crawling, Cohere API, Qdrant), so robust error handling is essential for reliability.

**Implementation Details**:
- Retry mechanisms for API calls with exponential backoff
- Graceful degradation when individual URLs fail
- Comprehensive logging for debugging
- Validation of API responses before processing

## Architecture Research

### Decision: Single file implementation with modular functions

**Rationale**: Following the requirement to implement everything in one file (main.py) while maintaining modularity through well-defined functions.

**Function Structure**:
- `get_all_urls()`: Extract all URLs from the Docusaurus site
- `extract_text_from_url()`: Extract clean text from a single URL
- `chunk_text()`: Split text into semantic chunks
- `create_collection()`: Initialize Qdrant collection
- `save_chunk_to_qdrant()`: Store chunk with embedding to Qdrant
- `main()`: Orchestrate the complete pipeline

## Performance Considerations

### Research Findings:
- Cohere API has rate limits that need to be respected
- Qdrant upsert operations can be batched for better performance
- Web crawling should implement reasonable delays to avoid overwhelming the target site
- Embedding generation can be parallelized to some extent while respecting API limits

## Security Considerations

### Research Findings:
- API keys should be stored in environment variables, not hardcoded
- Input validation needed for URLs to prevent malicious requests
- Rate limiting should be implemented to prevent abuse
- Proper authentication with Qdrant using API keys or other methods