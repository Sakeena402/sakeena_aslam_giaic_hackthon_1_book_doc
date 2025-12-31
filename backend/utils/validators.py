"""
Validator utilities for the Modular Retrieval System.

This module contains validation functions used across different modules.
"""

from typing import List, Dict, Any
from ..models.dataclasses import RetrievedChunk


def validate_retrieved_chunk_metadata(retrieved_chunk: RetrievedChunk) -> bool:
    """
    Validate the metadata of a retrieved chunk.

    Args:
        retrieved_chunk: The RetrievedChunk to validate

    Returns:
        bool: True if metadata is valid, False otherwise
    """
    required_fields = ["url", "title", "chunk_index", "content_preview"]
    metadata = retrieved_chunk.metadata

    for field in required_fields:
        if field not in metadata or not metadata[field]:
            return False

    return True


def validate_retrieved_chunks_metadata(retrieved_chunks: List[RetrievedChunk]) -> float:
    """
    Validate the metadata of multiple retrieved chunks.

    Args:
        retrieved_chunks: List of RetrievedChunk objects to validate

    Returns:
        float: Accuracy score (0-1) representing the ratio of valid chunks
    """
    if not retrieved_chunks:
        return 0.0

    total_chunks = len(retrieved_chunks)
    valid_chunks = 0

    for chunk in retrieved_chunks:
        if validate_retrieved_chunk_metadata(chunk):
            valid_chunks += 1

    return valid_chunks / total_chunks


def validate_similarity_scores(retrieved_chunks: List[RetrievedChunk], min_score: float = 0.0, max_score: float = 1.0) -> bool:
    """
    Validate that similarity scores are within the expected range.

    Args:
        retrieved_chunks: List of RetrievedChunk objects to validate
        min_score: Minimum allowed similarity score (default 0.0)
        max_score: Maximum allowed similarity score (default 1.0)

    Returns:
        bool: True if all scores are valid, False otherwise
    """
    for chunk in retrieved_chunks:
        if not (min_score <= chunk.similarity_score <= max_score):
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


def validate_chunk_list_integrity(content_chunks: List[str]) -> float:
    """
    Validate the integrity of multiple content chunks.

    Args:
        content_chunks: List of content chunks to validate

    Returns:
        float: Integrity score (0-1) representing the ratio of valid chunks
    """
    if not content_chunks:
        return 0.0

    total_chunks = len(content_chunks)
    valid_chunks = 0

    for chunk in content_chunks:
        if validate_chunk_integrity(chunk):
            valid_chunks += 1

    return valid_chunks / total_chunks


def validate_embedding_vector(embedding: List[float], expected_dimension: int = 1024, min_value: float = -2.0, max_value: float = 2.0) -> bool:
    """
    Validate an embedding vector for expected dimension and value ranges.

    Args:
        embedding: The embedding vector to validate
        expected_dimension: Expected number of dimensions (default 1024 for Cohere)
        min_value: Minimum allowed value in the embedding (default -2.0)
        max_value: Maximum allowed value in the embedding (default 2.0)

    Returns:
        bool: True if embedding is valid, False otherwise
    """
    if not embedding:
        return False

    if len(embedding) != expected_dimension:
        return False

    # Check that all values are within reasonable range
    for value in embedding:
        if not (min_value <= value <= max_value):
            return False

    return True


def validate_url_format(url: str) -> bool:
    """
    Validate that a URL has a proper format.

    Args:
        url: The URL to validate

    Returns:
        bool: True if URL format is valid, False otherwise
    """
    from urllib.parse import urlparse

    if not url:
        return False

    try:
        result = urlparse(url)
        return all([result.scheme, result.netloc])
    except Exception:
        return False


def validate_config_values(config: Any) -> Dict[str, Any]:
    """
    Validate configuration values and return a dictionary of validation results.

    Args:
        config: Configuration object with validation requirements

    Returns:
        Dict with validation results for each required field
    """
    results = {}

    # Validate required API keys and URLs
    results['cohere_api_key'] = bool(config.cohere_api_key)
    results['qdrant_url'] = validate_url_format(config.qdrant_url) if config.qdrant_url else False
    results['qdrant_api_key'] = bool(config.qdrant_api_key)
    results['book_base_url'] = validate_url_format(config.book_base_url) if config.book_base_url else False

    # Validate numeric settings
    results['chunk_size'] = config.chunk_size > 0
    results['overlap_size'] = config.overlap_size >= 0
    results['request_timeout'] = config.request_timeout > 0
    results['max_retries'] = config.max_retries > 0
    results['retry_delay'] = config.retry_delay > 0

    return results