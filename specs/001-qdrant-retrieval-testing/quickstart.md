# Quickstart: Qdrant Retrieval Testing

**Feature**: 001-qdrant-retrieval-testing
**Date**: 2025-12-11
**Related Plan**: [plan.md](./plan.md)

## Overview

This quickstart guide shows how to implement and test Qdrant retrieval functionality to verify that stored vectors can be accurately retrieved. The system ingests documents, generates embeddings, stores them in Qdrant, and validates retrieval accuracy.

## Prerequisites

- Python 3.11+
- UV package manager
- Cohere API key (for production, mock used for testing)
- Qdrant Cloud account and API key
- Access to target Docusaurus site: `https://hassaanghayas.github.io/physical-ai-robotics-textbook/`

### Environment Setup

1. **Create environment file**:
   ```bash
   # In backend/ directory
   cat > .env << EOF
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=https://your-cluster-url.qdrant.tech
   QDRANT_API_KEY=your_qdrant_api_key_here
   TARGET_DOCS_URL=https://hassaanghayas.github.io/physical-ai-robotics-textbook/
   EOF
   ```

2. **Install dependencies**:
   ```bash
   cd backend
   uv venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   uv pip install cohere qdrant-client requests beautifulsoup4 python-dotenv
   ```

## Implementation Steps

### Step 1: Corpus Ingestion (C001)

First, implement the corpus ingestion module that loads test documents into Qdrant:

```python
# In backend/main.py
def ingest_test_corpus(corpus_urls: List[str], collection_name: str = "rag_embedding") -> bool:
    """Ingest a known corpus for retrieval testing"""
    # Implementation for loading documents from corpus_urls
    # and storing them in Qdrant with proper metadata
    pass
```

### Step 2: Expected Value Recording (C002)

Record expected values for validation:

```python
# In backend/main.py
def record_expected_values(chunks: List[TextChunk]) -> Dict[str, Any]:
    """Record expected chunk texts, URLs, and chunk_ids for validation"""
    # Implementation to store baseline data for comparison
    pass
```

### Step 3: Query Preparation (C003)

Prepare test queries tied to corpus content:

```python
# In backend/main.py
def prepare_test_queries(expected_data: Dict[str, Any]) -> List[RetrievalQuery]:
    """Prepare test queries that should return specific results from corpus"""
    # Implementation to create queries that should match known content
    pass
```

### Step 4: Top-K Validation (C004)

Validate result count and ordering:

```python
# In backend/main.py
def validate_top_k_results(query: str, k: int, collection_name: str = "rag_embedding") -> Dict[str, Any]:
    """Validate that query returns exactly k results with proper ordering"""
    # Implementation to run query and verify result count and ordering
    pass
```

### Step 5: Chunk Integrity Validation (C005)

Verify content matches original:

```python
# In backend/main.py
def validate_chunk_integrity(retrieved_chunks: List[RetrievalResult], expected_chunks: List[str]) -> float:
    """Validate that retrieved text matches original chunks exactly or semantically"""
    # Implementation to compare retrieved vs expected content
    pass
```

### Step 6: Metadata Validation (C006)

Validate metadata correctness:

```python
# In backend/main.py
def validate_metadata_accuracy(results: List[RetrievalResult], expected_metadata: Dict[str, str]) -> float:
    """Validate that URL and chunk_id are returned correctly for each result"""
    # Implementation to compare retrieved vs expected metadata
    pass
```

### Step 7: End-to-End Pipeline Testing (C007)

Test complete pipeline:

```python
# In backend/main.py
def test_end_to_end_pipeline(test_queries: List[str], expected_results: List[Dict]) -> Dict[str, float]:
    """Run queries through full pipeline and verify clean JSON output"""
    # Implementation to execute complete workflow and validate output
    pass
```

## Testing Workflow

### Basic Test Run
```bash
# Run the complete test workflow
cd backend
uv run main.py --test-retrieval
```

### Targeted Testing
```bash
# Test specific functionality
uv run main.py --validate-top-k
uv run main.py --validate-chunk-integrity
uv run main.py --validate-metadata
```

### Custom Corpus Testing
```bash
# Test with custom URLs
uv run main.py --corpus-urls "['https://example.com/doc1', 'https://example.com/doc2']"
```

## Validation Criteria

### Top-K Validation
- Queries must return exactly the requested number of results (k)
- Results must be ordered by similarity score (descending)
- All results must have similarity scores above the threshold (typically 0.3)

### Chunk Integrity
- Retrieved content must match original text exactly or be semantically equivalent
- Minor formatting differences are acceptable if meaning is preserved
- Content length should be within 10% of expected length

### Metadata Accuracy
- URL field must match the source document URL
- Chunk ID must match the original chunk identifier
- All metadata fields must be present and correctly populated

### Performance Targets
- Query response time: <500ms for 95% of requests
- Success rate: >99% of queries return results without error
- Accuracy: >95% of top-k results are semantically relevant to query

## Troubleshooting

### Qdrant Connection Issues
```bash
# Verify Qdrant connectivity
curl -X GET "YOUR_QDRANT_URL/collections" -H "Api-Key: YOUR_API_KEY"
```

### Empty Results
- Check that documents were properly ingested into Qdrant
- Verify collection name matches between ingestion and retrieval
- Confirm query text is not empty or malformed

### Low Accuracy Results
- Verify embedding model consistency between ingestion and query
- Check that text preprocessing is identical for both ingestion and query
- Ensure query is semantically related to corpus content

## Example Usage

```python
# Example of using the retrieval testing functionality
from main import test_qdrant_retrieval_accuracy

# Test with default corpus
results = test_qdrant_retrieval_accuracy(
    corpus_urls=["https://hassaanghayas.github.io/physical-ai-robotics-textbook/"],
    test_queries=["What is physical AI?", "Explain humanoid robotics"],
    top_k=5
)

print(f"Top-k accuracy: {results['top_k_accuracy']:.2%}")
print(f"Chunk integrity: {results['chunk_integrity']:.2%}")
print(f"Metadata correctness: {results['metadata_correctness']:.2%}")
```

## Success Metrics

Upon successful implementation:

- ✅ 95%+ of queries return expected number of results
- ✅ 98%+ of retrieved chunks match original text or are semantically equivalent
- ✅ 100% of metadata fields (URL, chunk_id) are correct
- ✅ 95%+ of queries complete within 500ms
- ✅ End-to-end pipeline produces clean JSON output
- ✅ All test queries return expected results from known corpus