# Data Model: Embedding Pipeline Setup

**Feature**: Embedding Pipeline Setup
**Date**: 2025-12-11
**Branch**: 001-embedding-pipeline

## Overview

This document defines the data structures and entities for the embedding pipeline that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Core Entities

### Document
Represents a web page extracted from the Docusaurus site.

**Fields**:
- `id` (string): Unique identifier for the document (auto-generated)
- `url` (string): Original URL of the page
- `title` (string): Title of the page extracted from HTML
- `content` (string): Cleaned text content of the page
- `created_at` (datetime): Timestamp when the document was processed
- `updated_at` (datetime): Timestamp when the document was last updated

**Validation Rules**:
- `url` must be a valid URL format
- `content` must not be empty
- `id` must be unique across all documents

### TextChunk
Represents a semantic chunk of text from a document that will be embedded.

**Fields**:
- `id` (string): Unique identifier for the chunk (auto-generated)
- `document_id` (string): Reference to the parent document
- `content` (string): The text content of the chunk
- `chunk_index` (integer): Position of the chunk within the document
- `metadata` (dict): Additional metadata (headings, section, etc.)
- `created_at` (datetime): Timestamp when the chunk was created

**Validation Rules**:
- `document_id` must reference an existing document
- `content` must not exceed maximum token length for embedding model
- `chunk_index` must be non-negative

### Embedding
Represents a vector embedding of a text chunk stored in Qdrant.

**Fields**:
- `id` (string): Unique identifier for the embedding (Qdrant point ID)
- `chunk_id` (string): Reference to the source text chunk
- `document_id` (string): Reference to the parent document
- `embedding_vector` (list[float]): The vector representation of the text
- `metadata` (dict): Additional metadata including source URL, title, etc.
- `created_at` (datetime): Timestamp when the embedding was generated

**Validation Rules**:
- `embedding_vector` must have consistent dimensions based on the model used
- `chunk_id` must reference an existing text chunk
- `document_id` must reference an existing document

### QdrantPayload
The payload structure used when storing embeddings in Qdrant.

**Fields**:
- `content` (string): The text content of the chunk
- `url` (string): Source URL of the document
- `title` (string): Title of the document
- `document_id` (string): Reference to the source document
- `chunk_id` (string): Reference to the text chunk
- `chunk_index` (integer): Position of the chunk in the document
- `created_at` (datetime): When the embedding was created

## Relationships

```
Document (1) ─── (many) TextChunk (1) ─── (many) Embedding
```

- One Document can have many TextChunks
- One TextChunk results in one Embedding
- Embedding maintains references to both the TextChunk and Document

## Collection Schema

### Qdrant Collection: 'rag_embedding'

**Vector Configuration**:
- Vector size: 1024 (for Cohere's embedding model)
- Distance metric: Cosine

**Payload Schema**:
The collection will store embeddings with the following payload structure:
```json
{
  "content": "string",
  "url": "string",
  "title": "string",
  "document_id": "string",
  "chunk_id": "string",
  "chunk_index": "integer",
  "created_at": "datetime_string"
}
```

## State Transitions

### Document States
1. `pending` - URL discovered, not yet processed
2. `processing` - Content extraction in progress
3. `processed` - Content extracted and chunks created
4. `failed` - Error occurred during processing

### Embedding States
1. `pending` - Chunk created, embedding not yet generated
2. `embedding` - Embedding generation in progress
3. `stored` - Embedding successfully stored in Qdrant
4. `failed` - Error occurred during embedding or storage

## Indexing Strategy

### Qdrant Collection Indexes
- Payload index on `document_id` for efficient document-based queries
- Payload index on `url` for URL-based lookups
- Payload index on `chunk_index` for ordered retrieval

## Validation Rules Summary

1. **URL Validation**: All URLs must pass format validation
2. **Content Length**: Text chunks must be within embedding model limits
3. **Reference Integrity**: All foreign key references must exist
4. **Vector Dimensions**: Embeddings must have consistent dimensions
5. **Uniqueness**: Document IDs and embedding IDs must be unique
6. **Timestamps**: All records must have proper creation timestamps

## Constraints

1. Maximum chunk size: 512 tokens (configurable)
2. Minimum chunk size: 50 tokens (to maintain semantic meaning)
3. Vector dimension: Must match the Cohere embedding model being used
4. Metadata size: Payload in Qdrant should not exceed 10KB per record
5. Rate limiting: Embedding generation requests must respect Cohere API limits