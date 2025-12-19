Build RAG agent using OpenAI Agents SDK + FASTAPI with retrieval integration 

Goal: Create a backend Agent that can accept a user query, embed it, retrieve vectors from Qdrant

Success Criteria:
- FASTAPI exposes /ask endpoint
- Agent integrates Cohere embedding + Qdrant retrieval
- Responses includes: answers, sources, matched chunks
- Proper Error Handling (missing query, empty results)