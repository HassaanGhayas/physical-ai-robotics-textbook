"""
Retrieval service wrapper for RAG Agent.
Integrates with existing Qdrant retrieval functionality from main.py.
"""
import logging
from typing import List, Dict, Any
import time

# Import existing retrieval functionality
from main import retrieve, RetrievalResult

logger = logging.getLogger(__name__)


async def retrieve_documents(query: str, top_k: int = 5, collection_name: str = "rag_embedding") -> List[Dict[str, Any]]:
    """
    Retrieve relevant documents from Qdrant.

    This is an async wrapper around the existing retrieve() function from
    the 001-qdrant-retrieval-testing implementation.

    Args:
        query: User's natural language question
        top_k: Number of documents to retrieve (default: 5, max: 10)
        collection_name: Name of the Qdrant collection (default: rag_embedding)

    Returns:
        List of dictionaries containing retrieved chunks with metadata
    """
    start_time = time.time()

    try:
        # Call existing retrieve function (it's synchronous)
        results: List[RetrievalResult] = retrieve(
            query_text=query,
            top_k=top_k,
            collection_name=collection_name
        )

        # Format results for agent consumption
        formatted_results = []
        for r in results:
            formatted_results.append({
                "content": r.content,
                "url": r.url,
                "chunk_id": r.chunk_id,
                "document_id": r.document_id,
                "similarity_score": r.similarity_score,
                "position": r.position
            })

        end_time = time.time()
        retrieval_time = (end_time - start_time) * 1000

        logger.info(f"Retrieved {len(formatted_results)} documents in {retrieval_time:.2f}ms for query: '{query[:50]}...'")

        return formatted_results

    except Exception as e:
        logger.error(f"Error in retrieve_documents: {str(e)}")
        raise


def get_retrieval_time_ms() -> float:
    """
    Get the time taken for the last retrieval operation.
    Used for performance tracking and metadata.
    """
    # This would be tracked in a more sophisticated way in production
    # For now, return 0 as it's tracked inline in retrieve_documents
    return 0.0
