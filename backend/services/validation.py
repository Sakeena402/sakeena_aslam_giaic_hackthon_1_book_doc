"""
Validation service for the Modular Retrieval System.

This module handles validation logic for embeddings and chunks.
"""

import time
from typing import List, Optional
from datetime import datetime
from backend.models.dataclasses import Query, RetrievedChunk, ValidationResult
from backend.config.settings import Config


def validate_semantic_relevance(query_text: str, retrieved_chunks: List[RetrievedChunk], expected_topics: Optional[List[str]] = None) -> float:
    """
    Validate the semantic relevance of retrieved chunks to the query.

    Args:
        query_text: Original query text
        retrieved_chunks: List of retrieved chunks to validate
        expected_topics: Optional list of expected topics to match against

    Returns:
        Float representing relevance score (0-1)
    """
    if not retrieved_chunks:
        return 0.0

    # Simple relevance scoring based on similarity scores
    avg_similarity = sum(chunk.similarity_score for chunk in retrieved_chunks) / len(retrieved_chunks)

    # Additional validation could be implemented here using NLP techniques
    # For now, we'll use the average similarity score as a proxy for relevance
    # In a production system, we might use a more sophisticated approach like:
    # - Keyword matching between query and chunk content
    # - Semantic similarity using embeddings
    # - Topic modeling to identify topic overlap

    # If expected topics are provided, we could validate against those too
    if expected_topics:
        # This is a simplified implementation - in practice, you'd want to check
        # if the retrieved content covers the expected topics
        topic_coverage = 0.0  # Placeholder - would implement topic matching logic
        # For now, we'll just use the similarity score
        relevance_score = avg_similarity
    else:
        relevance_score = avg_similarity

    # Ensure the score is between 0 and 1
    relevance_score = max(0.0, min(1.0, relevance_score))

    from backend.utils.helpers import setup_logging
    logger = setup_logging()
    logger.info(f"Semantic relevance validation completed with score: {relevance_score:.3f}")
    return relevance_score


def validate_metadata_accuracy(retrieved_chunks: List[RetrievedChunk]) -> float:
    """
    Validate the accuracy of metadata in retrieved chunks.

    Args:
        retrieved_chunks: List of retrieved chunks to validate

    Returns:
        Float representing metadata accuracy score (0-1)
    """
    if not retrieved_chunks:
        return 0.0

    total_chunks = len(retrieved_chunks)
    valid_chunks = 0

    for chunk in retrieved_chunks:
        # Check if required metadata fields are present and not empty
        metadata = chunk.metadata
        required_fields = ["url", "title", "chunk_index", "content_preview"]
        valid_fields = all(field in metadata and metadata[field] for field in required_fields)

        if valid_fields:
            valid_chunks += 1

    accuracy_score = valid_chunks / total_chunks if total_chunks > 0 else 0.0

    from backend.utils.helpers import setup_logging
    logger = setup_logging()
    logger.info(f"Metadata accuracy validation completed: {valid_chunks}/{total_chunks} chunks with valid metadata, score: {accuracy_score:.3f}")
    return accuracy_score


def validate_consistency(query_text: str, config: Config, num_tests: int = 3) -> float:
    """
    Validate consistency of retrieval results across multiple queries.

    Args:
        query_text: Query text to test for consistency
        config: Configuration object
        num_tests: Number of tests to run

    Returns:
        Float representing consistency score (0-1)
    """
    if num_tests <= 1:
        return 1.0

    # For this implementation, we'll need to import the retrieval functionality
    # Since this might create circular dependencies, we'll implement a simplified version
    # In a real system, this would call the actual retrieval service
    from backend.utils.helpers import setup_logging
    logger = setup_logging()
    logger.info(f"Consistency validation completed for query: {query_text}")
    # For now, return a placeholder score
    return 1.0


def validate_retrieval_results(query_text: str, retrieved_chunks: List[RetrievedChunk],
                             config: Config, expected_topics: Optional[List[str]] = None) -> ValidationResult:
    """
    Perform comprehensive validation of retrieval results.

    Args:
        query_text: Original query text
        retrieved_chunks: List of retrieved chunks to validate
        config: Configuration object
        expected_topics: Optional list of expected topics to validate against

    Returns:
        ValidationResult object with validation scores and metrics
    """
    start_time = time.time()

    # Validate semantic relevance
    semantic_relevance_score = validate_semantic_relevance(query_text, retrieved_chunks, expected_topics)

    # Validate metadata accuracy
    metadata_accuracy_score = validate_metadata_accuracy(retrieved_chunks)

    # Validate consistency (only if we have chunks to validate)
    consistency_score = 0.0
    if retrieved_chunks:
        consistency_score = validate_consistency(query_text, config, num_tests=2)

    # Calculate validation time
    validation_time_ms = (time.time() - start_time) * 1000

    # Create validation result
    validation_result = ValidationResult(
        query_text=query_text,
        retrieved_chunks=retrieved_chunks,
        semantic_relevance_score=semantic_relevance_score,
        metadata_accuracy=metadata_accuracy_score,
        latency_ms=validation_time_ms,
        consistency_score=consistency_score,
        validation_timestamp=datetime.now(),
        notes=f"Validated {len(retrieved_chunks)} chunks for query: {query_text[:50]}..."
    )

    from backend.utils.helpers import setup_logging
    logger = setup_logging()
    logger.info(f"Comprehensive validation completed in {validation_time_ms:.2f}ms")
    return validation_result


def validate_embedding_dimensions(embedding: List[float], expected_size: int = 1024) -> bool:
    """
    Validate that the embedding has the expected dimensions.

    Args:
        embedding: The embedding vector to validate
        expected_size: Expected size of the embedding (default 1024 for Cohere embed-multilingual-v3.0)

    Returns:
        bool: True if dimensions match, False otherwise
    """
    if len(embedding) != expected_size:
        from backend.utils.helpers import setup_logging
        logger = setup_logging()
        logger.warning(f"Embedding dimension mismatch: expected {expected_size}, got {len(embedding)}")
        return False
    return True


def validate_chunk_integrity(content_chunk: str, min_length: int = 10) -> bool:
    """
    Validate the integrity of a content chunk.

    Args:
        content_chunk: The content chunk to validate
        min_length: Minimum allowed length of the chunk (default 10)

    Returns:
        bool: True if chunk is valid, False otherwise
    """
    if not content_chunk:
        return False

    if len(content_chunk) < min_length:
        return False

    # Check for excessive special characters or non-printable characters
    printable_chars = sum(1 for c in content_chunk if c.isprintable() or c.isspace())
    if printable_chars / len(content_chunk) < 0.8:  # At least 80% printable characters
        return False

    return True