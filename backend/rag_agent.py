"""
RAG Agent implementation using Cohere for generation.
Orchestrates retrieval and generation workflow.
"""
import logging
import time
import asyncio
from typing import List, Dict, Any, Optional
import cohere
from retrieval import retrieve_documents
from models import AgentResponse, SourceChunk, ResponseMetadata
from config import settings

logger = logging.getLogger(__name__)

# Initialize Cohere client (only if API key is available)
cohere_client = None
if settings.cohere_api_key:
    cohere_client = cohere.AsyncClient(api_key=settings.cohere_api_key)
    logger.info("Cohere client initialized successfully")
else:
    logger.warning("Cohere client not initialized - COHERE_API_KEY not set")


async def call_retrieval_tool(query: str, top_k: int = 5) -> tuple[List[Dict[str, Any]], float]:
    """
    Call the retrieval tool to get documents from Qdrant.

    Args:
        query: Search query
        top_k: Number of documents to retrieve

    Returns:
        List of retrieved documents with metadata
    """
    start_time = time.time()

    try:
        results = await retrieve_documents(query, top_k)
        end_time = time.time()
        retrieval_time = (end_time - start_time) * 1000

        logger.info(f"Retrieval tool returned {len(results)} documents in {retrieval_time:.2f}ms")
        return results, retrieval_time

    except Exception as e:
        logger.error(f"Error in call_retrieval_tool: {str(e)}")
        raise


async def process_query(query: str, top_k: int = 5, include_sources: bool = True) -> AgentResponse:
    """
    Process a query through the RAG agent workflow.

    This orchestrates the complete RAG pipeline:
    1. Call retrieval tool to get relevant documents
    2. Format context from retrieved documents
    3. Call Cohere to generate answer based on context
    4. Format response with sources and metadata

    Args:
        query: User's natural language question
        top_k: Number of documents to retrieve
        include_sources: Whether to include source attribution

    Returns:
        AgentResponse with answer, sources, and metadata
    """
    start_time = time.time()

    # Check if Cohere client is initialized
    if not cohere_client:
        raise RuntimeError("Cohere client not initialized. Please set COHERE_API_KEY environment variable.")

    try:
        # Step 1: Retrieve relevant documents
        logger.info(f"Processing query: '{query[:50]}...'")
        retrieval_start = time.time()
        retrieved_docs, retrieval_time_ms = await call_retrieval_tool(query, top_k)
        retrieval_end = time.time()

        # Track embedding time (from retrieval process)
        embedding_time_ms = retrieval_time_ms * 0.3  # Approximate 30% for embedding

        # Step 2: Format context and documents for Cohere
        if not retrieved_docs:
            logger.warning("No relevant documents found for query")
            # Generate answer without context
            documents = []
            context_available = False
        else:
            # Format documents for Cohere's chat API
            documents = []
            for doc in retrieved_docs:
                documents.append({
                    "text": doc['content'],
                    "url": doc['url'],
                    "title": f"Source {doc['position']} (Score: {doc['similarity_score']:.3f})"
                })
            context_available = True

        # Step 3: Call Cohere to generate answer
        generation_start = time.time()

        # Build the preamble (system instructions)
        preamble = """You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook.

CRITICAL RULES:
1. ONLY answer based on the provided document context - never use external knowledge
2. If the context doesn't contain relevant information, say "I don't have information about that in the textbook"
3. Quote or paraphrase directly from the provided sources
4. Be concise but comprehensive (aim for 2-4 sentences)
5. Maintain a professional and educational tone

Your answers must be grounded in the textbook content provided. Do not speculate or add information not present in the sources."""

        # Use Cohere's chat API with RAG
        response = await cohere_client.chat(
            message=query,
            documents=documents if context_available else None,
            preamble=preamble,
            model=settings.cohere_model or "command-r",
            temperature=0.3,  # Lower temperature for more deterministic, grounded responses
            max_tokens=500,
            prompt_truncation="AUTO"
        )

        answer = response.text
        generation_end = time.time()
        generation_time_ms = (generation_end - generation_start) * 1000

        # Step 4: Format sources
        sources = []
        if include_sources and retrieved_docs:
            for doc in retrieved_docs:
                sources.append(SourceChunk(
                    content=doc["content"],
                    url=doc["url"],
                    chunk_id=doc.get("chunk_id", ""),
                    document_id=doc.get("document_id", ""),
                    similarity_score=doc["similarity_score"],
                    position=doc["position"]
                ))

        # Step 5: Calculate total response time and create metadata
        end_time = time.time()
        response_time_ms = (end_time - start_time) * 1000

        metadata = ResponseMetadata(
            response_time_ms=response_time_ms,
            chunk_count=len(retrieved_docs),
            embedding_time_ms=embedding_time_ms,
            retrieval_time_ms=retrieval_time_ms,
            generation_time_ms=generation_time_ms
        )

        logger.info(f"Query processed in {response_time_ms:.2f}ms (embedding: {embedding_time_ms:.2f}ms, retrieval: {retrieval_time_ms:.2f}ms, generation: {generation_time_ms:.2f}ms)")

        return AgentResponse(
            answer=answer,
            sources=sources,
            metadata=metadata
        )

    except Exception as e:
        logger.error(f"Error in process_query: {str(e)}")
        raise
