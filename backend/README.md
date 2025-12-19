# RAG Agent API

Backend API for RAG (Retrieval-Augmented Generation) agent using OpenAI, Cohere embeddings, and Qdrant retrieval.

## Features

- **POST /ask** - Ask questions and receive AI-generated answers with source citations
- **GET /health** - Check service health and external API connectivity
- **GET /** - API information and available endpoints

## Prerequisites

- Python 3.11+
- OpenAI API key (required for answer generation)
- Cohere API key (for embedding generation)
- Qdrant Cloud account with populated collection
- Existing Qdrant collection `rag_embedding` from 001-qdrant-retrieval-testing

## Installation

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   # Or use UV:
   uv pip install fastapi uvicorn openai cohere qdrant-client pydantic pydantic-settings python-dotenv httpx tenacity pybreaker
   ```

2. **Configure environment**:
   ```bash
   cp .env.example .env
   # Edit .env and add your API keys:
   # OPENAI_API_KEY=your_key_here
   # COHERE_API_KEY=your_key_here
   # QDRANT_URL=your_qdrant_url
   # QDRANT_API_KEY=your_qdrant_key
   ```

## Running the API

```bash
python api.py
# Or use uvicorn directly:
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`

## API Usage

### Ask a Question

```bash
curl -X POST "http://localhost:8000/ask" \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?",
    "top_k": 5,
    "include_sources": true
  }'
```

**Response**:
```json
{
  "answer": "Physical AI refers to artificial intelligence systems...",
  "sources": [
    {
      "content": "...",
      "url": "https://...",
      "chunk_id": "...",
      "document_id": "...",
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

### Check Health

```bash
curl "http://localhost:8000/health"
```

## Architecture

The RAG agent follows this workflow:

1. **Query Processing**: Validate user input
2. **Retrieval**: Get relevant documents from Qdrant using Cohere embeddings
3. **Context Formatting**: Prepare retrieved content for generation
4. **Generation**: Use OpenAI to synthesize answer from context
5. **Response Formatting**: Return structured JSON with answer, sources, and metadata

## Error Handling

The API implements comprehensive error handling:

- **400 Bad Request**: Invalid input (empty query, length exceeded, invalid parameters)
- **500 Internal Server Error**: Unexpected errors during processing
- **503 Service Unavailable**: External service failures (OpenAI, Cohere, Qdrant)
- **504 Gateway Timeout**: Request exceeded 5-second timeout

Retry logic with exponential backoff (3 attempts) handles transient failures.
Circuit breakers prevent cascading failures when services degrade.

## Performance

- **Target**: Sub-5-second response time (95th percentile)
- **Typical**: 3-4 seconds end-to-end
  - Embedding: 250ms
  - Retrieval: 150ms
  - Generation: 2800ms
- **Concurrency**: Supports 50 concurrent users

## Development

### Running Tests

```bash
pytest tests/ -v
pytest tests/test_api.py -v
pytest tests/test_agent.py -v
```

### API Documentation

Visit `http://localhost:8000/docs` for interactive Swagger/OpenAPI documentation.

## Troubleshooting

### "OpenAI client not initialized"
- Ensure `OPENAI_API_KEY` is set in your .env file
- Restart the server after updating environment variables

### "No relevant documents found"
- Check that Qdrant collection `rag_embedding` has documents
- Verify documents were ingested from 001-qdrant-retrieval-testing

### Slow response times
- Check network latency to external APIs
- Reduce `top_k` if retrieval is slow
- Monitor individual timing metrics in response metadata

## Related Features

- **001-qdrant-retrieval-testing**: Provides the underlying Qdrant retrieval functionality
- See `specs/002-rag-agent/` for complete specification and planning documents
