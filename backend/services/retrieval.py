"""
Retrieval service for the Modular Retrieval System.

This module handles query processing and similarity search.
"""

from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from backend.models.dataclasses import Query, RetrievedChunk, RetrievalRequest, RetrievalResponse
from backend.config.settings import Config


class QdrantRetriever:
    """Class to handle vector retrieval from Qdrant."""

    def __init__(self, config: Config):
        self.config = config
        # Create Qdrant client with connection details from config
        self.client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            prefer_grpc=False,
            timeout=config.request_timeout
        )
        self.collection_name = config.qdrant_collection_name

    def search_similar(self, query_embedding: List[float], top_k: int = 5, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
        """
        Perform similarity search in Qdrant and return relevant content chunks.

        Args:
            query_embedding: The embedding vector to search for
            top_k: Number of top results to return
            similarity_threshold: Minimum similarity threshold for results

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Perform the search in Qdrant - using the correct method name for newer Qdrant versions
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,
            ).points

            retrieved_chunks = []
            for result in search_results:
                # Extract content from payload
                payload = result.payload
                metadata = {
                    "url": payload.get("url", ""),
                    "chapter": payload.get("chapter", ""),
                    "section": payload.get("section", ""),
                    "title": payload.get("title", ""),
                    "chunk_index": payload.get("chunk_index", 0),
                    "content_preview": payload.get("content_preview", ""),
                    "heading_context": payload.get("heading_context", "")
                }

                # Create RetrievedChunk object
                retrieved_chunk = RetrievedChunk(
                    id=result.id,
                    content=payload.get("content", payload.get("content_preview", "")),
                    similarity_score=result.score,
                    metadata=metadata
                )

                retrieved_chunks.append(retrieved_chunk)

            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant with similarity threshold {similarity_threshold}")
            return retrieved_chunks

        except Exception as e:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Similarity search failed: {str(e)}")
            raise

    def search_with_filters(self, query_embedding: List[float], filters: Optional[Filter] = None,
                           top_k: int = 5, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
        """
        Perform similarity search in Qdrant with optional filters.

        Args:
            query_embedding: The embedding vector to search for
            filters: Optional filters to apply to the search
            top_k: Number of top results to return
            similarity_threshold: Minimum similarity threshold for results

        Returns:
            List of RetrievedChunk objects
        """
        try:
            # Perform the search in Qdrant with filters
            search_results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                score_threshold=similarity_threshold,
                query_filter=filters
            ).points

            retrieved_chunks = []
            for result in search_results:
                # Extract content from payload
                payload = result.payload
                metadata = {
                    "url": payload.get("url", ""),
                    "chapter": payload.get("chapter", ""),
                    "section": payload.get("section", ""),
                    "title": payload.get("title", ""),
                    "chunk_index": payload.get("chunk_index", 0),
                    "content_preview": payload.get("content_preview", ""),
                    "heading_context": payload.get("heading_context", "")
                }

                # Create RetrievedChunk object
                retrieved_chunk = RetrievedChunk(
                    id=result.id,
                    content=payload.get("content", payload.get("content_preview", "")),
                    similarity_score=result.score,
                    metadata=metadata
                )

                retrieved_chunks.append(retrieved_chunk)

            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Retrieved {len(retrieved_chunks)} chunks from Qdrant with filters and similarity threshold {similarity_threshold}")
            return retrieved_chunks

        except Exception as e:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Similarity search with filters failed: {str(e)}")
            raise


def simple_retrieve_chunks(query_text: str, config: Config, top_k: int = 5, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
    """
    Simple retrieval function that accepts a query string and returns relevant content chunks.

    Args:
        query_text: The query text to search for
        config: Configuration object with API keys and settings
        top_k: Number of top results to return
        similarity_threshold: Minimum similarity threshold for results

    Returns:
        List of RetrievedChunk objects
    """
    # Import cohere for embedding generation
    import cohere
    import time

    # Create Cohere client
    cohere_client = cohere.Client(config.cohere_api_key)

    # Retry mechanism with rate limiting for query embedding
    max_retries = 3
    base_delay = 1.5  # 1.5 seconds as requested

    query_embedding = None

    for attempt in range(max_retries):
        try:
            # Generate embedding for the query
            response = cohere_client.embed(
                texts=[query_text],
                model=config.retrieval_model_name,
                input_type="search_query"
            )

            query_embedding = response.embeddings[0]
            break  # Success, exit retry loop

        except Exception as e:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()

            # Check if it's a rate limit error
            if "429" in str(e) or "TooManyRequests" in str(e) or "rate limit" in str(e).lower():
                logger.warning(f"Rate limit exceeded on query embedding attempt {attempt + 1}, waiting {base_delay}s before retry...")
                time.sleep(base_delay)
                base_delay *= 2  # Exponential backoff
            elif "Connection" in str(e) or "connection" in str(e).lower() or "network" in str(e).lower():
                logger.warning(f"Network error on query embedding attempt {attempt + 1}, waiting {base_delay}s before retry...")
                time.sleep(base_delay)
                base_delay *= 2  # Exponential backoff
            else:
                logger.error(f"Failed to generate query embedding: {str(e)}")
                if attempt == max_retries - 1:  # Last attempt
                    raise  # Re-raise the exception if all retries failed

    if query_embedding is None:
        from backend.utils.helpers import setup_logging
        logger = setup_logging()
        logger.error("Failed to generate query embedding after all retry attempts")
        return []

    # Initialize the retriever
    retriever = QdrantRetriever(config)

    # Perform similarity search
    retrieved_chunks = retriever.search_similar(
        query_embedding=query_embedding,
        top_k=top_k,
        similarity_threshold=similarity_threshold
    )

    from backend.utils.helpers import setup_logging
    logger = setup_logging()
    logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: {query_text}")

    return retrieved_chunks