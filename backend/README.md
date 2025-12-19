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

## Deployment to Hugging Face Spaces

### Prerequisites
- Hugging Face account
- Repository secrets configured (COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL)

### Deployment Steps

1. **Create Hugging Face Space**:
   - Go to https://huggingface.co/new-space
   - Name: `physical-ai-robotics-backend`
   - SDK: Docker (or Gradio if using app.py entrypoint)
   - Visibility: Public

2. **Link to GitHub**:
   - In Space Settings → Repository → Link to GitHub
   - Select repository: `HassaanGhayas/physical-ai-robotics-textbook`
   - Sync directory: `backend/`

3. **Configure Environment Variables**:
   - In Space Settings → Variables, add:
     - `COHERE_API_KEY`: Your Cohere API key
     - `QDRANT_API_KEY`: Your Qdrant API key
     - `QDRANT_URL`: Your Qdrant cluster URL (e.g., https://xxx.cloud.qdrant.io)
     - `COHERE_MODEL`: command-r-08-2024
     - `QDRANT_COLLECTION_NAME`: rag_embedding
     - `TARGET_DOCS_URL`: https://hassaanghayas.github.io/physical-ai-robotics-textbook/
     - `TOP_K`: 5
     - `LOG_LEVEL`: INFO

4. **Push to Deploy**:
   - Push changes to `001-book-creation` branch
   - Hugging Face will automatically build and deploy
   - Build time: 5-10 minutes

5. **Verify Deployment**:
   ```bash
   # Check health endpoint
   curl https://your-space-name.hf.space/health

   # Test query
   curl -X POST "https://your-space-name.hf.space/ask" \
     -H "Content-Type: application/json" \
     -d '{"query": "What is physical AI?", "top_k": 5}'
   ```

### Production URL
Once deployed, your backend will be available at:
```
https://physical-ai-robotics-backend.hf.space
```

### CORS Configuration
The API is configured to accept requests from:
- `http://localhost:3000` (development)
- `https://hassaanghayas.github.io` (production GitHub Pages)

### Monitoring
- Check Space logs for deployment issues
- Monitor /health endpoint for service status
- Track response times in API metadata

## Related Features

- **001-qdrant-retrieval-testing**: Provides the underlying Qdrant retrieval functionality
- See `specs/002-rag-agent/` for complete specification and planning documents
