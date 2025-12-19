# Data Model: Qdrant Retrieval Testing

**Feature**: 001-qdrant-retrieval-testing
**Date**: 2025-12-11
**Related Spec**: [spec.md](./spec.md)

## Overview

This document defines the data structures for the Qdrant retrieval testing functionality, focusing on how documents, chunks, and embeddings are represented for the purpose of validating retrieval accuracy.

## Core Entities

### Document
Represents a source document that will be processed and stored in the vector database.

**Fields**:
- `id` (string): Unique identifier for the document
- `url` (string): Source URL of the document
- `title` (string): Title of the document
- `content` (string): Full text content of the document
- `created_at` (string): ISO timestamp when document was processed
- `status` (string): Processing status (pending, processed, failed)

**Validation Rules**:
- `id` must be unique across all documents
- `url` must be a valid URL format
- `content` must not be empty
- `status` must be one of the allowed values

### TextChunk
Represents a semantic chunk of text from a document that will be converted to an embedding.

**Fields**:
- `id` (string): Unique identifier for the chunk (usually derived from document ID and chunk index)
- `document_id` (string): Reference to the parent document
- `content` (string): The text content of the chunk
- `chunk_index` (integer): Position of the chunk within the document (0-indexed)
- `metadata` (dict): Additional metadata including source URL, title, etc.
- `created_at` (string): ISO timestamp when chunk was created

**Validation Rules**:
- `document_id` must reference an existing document
- `content` must not be empty
- `chunk_index` must be non-negative
- `metadata` must contain at least the source URL

### RetrievalQuery
Represents a query that will be sent to the vector database for testing retrieval.

**Fields**:
- `id` (string): Unique identifier for the query
- `query_text` (string): The text to search for
- `top_k` (integer): Number of results to retrieve
- `expected_results` (list): List of expected chunk IDs that should be returned
- `created_at` (string): ISO timestamp when query was created

**Validation Rules**:
- `query_text` must not be empty
- `top_k` must be a positive integer
- `expected_results` must be valid chunk IDs if specified

### RetrievalResult
Represents a result returned from the vector database during retrieval testing.

**Fields**:
- `id` (string): Qdrant point ID for this result
- `content` (string): The retrieved text content
- `url` (string): Source URL of the retrieved content
- `chunk_id` (string): ID of the original chunk
- `document_id` (string): ID of the original document
- `similarity_score` (float): Cosine similarity score between query and result
- `position` (integer): Rank position in the results (1-indexed)
- `query_id` (string): Reference to the query that generated this result

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `position` must be a positive integer
- `query_id` must reference an existing query

### ValidationResult
Represents the results of a validation test on retrieval functionality.

**Fields**:
- `id` (string): Unique identifier for the validation result
- `test_name` (string): Name of the test being validated
- `query_id` (string): Reference to the query being tested
- `passed` (boolean): Whether the validation passed
- `expected_chunk_ids` (list): List of chunk IDs that were expected to be returned
- `actual_chunk_ids` (list): List of chunk IDs that were actually returned
- `top_k_accuracy` (float): Accuracy of top-k result count
- `chunk_integrity` (float): Percentage of chunks that matched expected content
- `metadata_correctness` (float): Percentage of metadata fields that were correct
- `details` (dict): Detailed information about the validation
- `created_at` (string): ISO timestamp when validation was performed

**Validation Rules**:
- `top_k_accuracy`, `chunk_integrity`, and `metadata_correctness` must be between 0.0 and 1.0
- `passed` must correspond to whether the scores meet minimum thresholds

## Relationships

```
Document (1) ─── (many) TextChunk (1) ─── (many) RetrievalResult
```

- One Document can have many TextChunks
- One TextChunk can appear in many RetrievalResults (from different queries)
- One RetrievalQuery can generate many RetrievalResults
- One ValidationResult can reference many RetrievalResults

## Collections & Indexes

### Qdrant Collection: 'rag_embedding'

**Vector Configuration**:
- Vector size: 1024 (for Cohere's embedding model)
- Distance metric: Cosine

**Payload Schema**:
```json
{
  "content": "string",
  "url": "string",
  "chunk_id": "string",
  "document_id": "string",
  "title": "string",
  "chunk_index": "integer",
  "created_at": "string"
}
```

**Indexes**:
- Payload index on `document_id` for efficient document-based queries
- Payload index on `url` for URL-based lookups
- Payload index on `chunk_index` for ordered retrieval

## State Transitions

### Document States
1. `pending` - Document URL identified, not yet processed
2. `processing` - Content extraction in progress
3. `processed` - Content extracted and chunks created
4. `failed` - Error occurred during processing

### Chunk States
1. `pending` - Chunk created, not yet embedded
2. `embedding` - Embedding generation in progress
3. `embedded` - Embedding generated, not yet stored
4. `stored` - Chunk with embedding stored in Qdrant
5. `failed` - Error occurred during embedding or storage

### Validation States
1. `not_started` - Validation test not yet executed
2. `in_progress` - Validation test running
3. `passed` - Validation test completed successfully
4. `failed` - Validation test completed with failures
5. `partial` - Validation test partially completed

## Validation Rules Summary

1. **Content Integrity**: Retrieved content must match original or be semantically equivalent
2. **Metadata Accuracy**: All metadata fields (URL, chunk_id, document_id) must be preserved
3. **Result Count**: Top-k queries must return the specified number of results when available
4. **Similarity Threshold**: Retrieved results must have sufficient similarity to query
5. **ID Consistency**: Qdrant point IDs must be consistent and traceable to source
6. **Timestamp Validity**: All timestamps must be in ISO format and reasonable

## Constraints

1. Maximum chunk size: 512 tokens (configurable)
2. Minimum chunk size: 50 tokens (to maintain semantic meaning)
3. Vector dimension: Must match the embedding model being used (1024 for Cohere)
4. Metadata size: Payload in Qdrant should not exceed 10KB per record
5. Similarity threshold: Results should have >0.3 similarity score to be considered relevant