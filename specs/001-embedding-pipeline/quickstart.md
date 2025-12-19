# Quickstart: Embedding Pipeline Setup

**Feature**: Embedding Pipeline Setup
**Date**: 2025-12-11
**Branch**: 001-embedding-pipeline

## Overview

This guide provides quick setup instructions for the embedding pipeline that extracts text from deployed Docusaurus URLs, generates embeddings using Cohere, and stores them in Qdrant for RAG-based retrieval.

## Prerequisites

- Python 3.11 or higher
- UV package manager installed
- Cohere API key
- Qdrant instance (cloud or local) with API key
- Access to the target Docusaurus site (https://hassaanghayas.github.io/physical-ai-robotics-textbook/)

## Setup Steps

### 1. Clone and Navigate to Backend Directory

```bash
# If you don't have a backend directory yet:
mkdir backend
cd backend
```

### 2. Initialize Project with UV

```bash
# Create pyproject.toml
uv init

# Or if starting fresh:
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
```

### 3. Install Dependencies

```bash
uv add cohere qdrant-client requests beautifulsoup4 python-dotenv fastapi uvicorn
```

Or add to your `pyproject.toml`:

```toml
[project]
name = "embedding-pipeline"
version = "0.1.0"
description = "Embedding pipeline for Docusaurus content"
requires-python = ">=3.11"

dependencies = [
    "cohere>=5.0.0",
    "qdrant-client>=1.9.0",
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "python-dotenv>=1.0.0",
    "fastapi>=0.104.0",
    "uvicorn>=0.24.0",
]
```

### 4. Create Environment File

Create `.env` file in the backend directory:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., https://your-cluster-url.qdrant.tech
QDRANT_API_KEY=your_qdrant_api_key_here
TARGET_URL=https://hassaanghayas.github.io/physical-ai-robotics-textbook/
CHUNK_SIZE=512
CHUNK_OVERLAP=50
```

### 5. Create the Main Application

Create `main.py` with the complete implementation as specified:

```python
# This will contain the complete implementation with all required functions:
# - get_all_urls()
# - extract_text_from_url()
# - chunk_text()
# - create_collection()
# - save_chunk_to_qdrant()
# - main() function orchestrating the pipeline
```

### 6. Run the Pipeline

```bash
cd backend
python main.py
```

## Configuration Options

### Environment Variables

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant authentication
- `TARGET_URL`: The Docusaurus site URL to process
- `CHUNK_SIZE`: Maximum size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)

### Adjustable Parameters

- Chunk size: Modify the maximum tokens per chunk
- Chunk overlap: Set overlap to maintain context between chunks
- Batch size: Number of chunks to process simultaneously
- Rate limiting: Delays between API calls to respect limits

## Verification Steps

1. **Check Qdrant Collection**: Verify that the 'rag_embedding' collection exists in Qdrant
2. **Check Document Count**: Confirm that documents were processed from the target URL
3. **Test Search**: Perform a test search to verify embeddings are retrievable
4. **Log Review**: Check logs for any errors or warnings during processing

## Common Issues and Solutions

### API Rate Limits
- **Issue**: Cohere API rate limiting
- **Solution**: Implement exponential backoff and respect API limits

### Network Issues
- **Issue**: Connection timeouts during crawling
- **Solution**: Increase timeout values and implement retry logic

### Qdrant Connection
- **Issue**: Unable to connect to Qdrant
- **Solution**: Verify URL and API key in environment variables

## Next Steps

1. After successful pipeline execution, the embeddings will be stored in Qdrant
2. Implement a search endpoint to retrieve similar content
3. Create a RAG application that uses these embeddings
4. Add monitoring and logging for production use

## Development Workflow

1. **Local Testing**: Test with a subset of URLs first
2. **Environment Setup**: Use different environments (dev/staging/prod)
3. **Monitoring**: Add metrics to track pipeline performance
4. **Error Handling**: Implement comprehensive error handling and recovery