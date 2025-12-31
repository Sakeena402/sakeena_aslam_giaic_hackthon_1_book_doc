"""
Utility functions for the Modular Retrieval System.

This module contains common utility functions used across different modules.
"""

import time
import logging
from typing import Callable, Any
from functools import wraps


def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    return logging.getLogger(__name__)


def retry_with_backoff(func, max_retries=3, base_delay=1.0, backoff_factor=2.0):
    """
    Decorator to retry a function with exponential backoff.

    Args:
        func: The function to retry
        max_retries: Maximum number of retry attempts
        base_delay: Initial delay between retries
        backoff_factor: Factor by which delay increases after each retry
    """
    def wrapper(*args, **kwargs):
        delay = base_delay
        last_exception = None

        for attempt in range(max_retries):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                last_exception = e
                if attempt == max_retries - 1:
                    break
                logger = setup_logging()
                logger.warning(f"Attempt {attempt + 1} failed: {str(e)}. Retrying in {delay} seconds...")
                time.sleep(delay)
                delay *= backoff_factor

        logger = setup_logging()
        logger.error(f"All {max_retries} attempts failed. Last error: {str(last_exception)}")
        raise last_exception

    return wrapper


def validate_embedding_dimensions(embedding: list, expected_size: int = 1024) -> bool:
    """
    Validate that the embedding has the expected dimensions.

    Args:
        embedding: The embedding vector to validate
        expected_size: Expected size of the embedding (default 1024 for Cohere embed-multilingual-v3.0)

    Returns:
        bool: True if dimensions match, False otherwise
    """
    if len(embedding) != expected_size:
        logger = setup_logging()
        logger.warning(f"Embedding dimension mismatch: expected {expected_size}, got {len(embedding)}")
        return False
    return True


def time_it(func: Callable) -> Callable:
    """
    Decorator to time function execution and log the duration.

    Args:
        func: The function to time

    Returns:
        Callable: The timed function
    """
    @wraps(func)
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        duration = (end_time - start_time) * 1000  # Convert to milliseconds
        logger = setup_logging()
        logger.info(f"{func.__name__} executed in {duration:.2f}ms")
        return result
    return wrapper


def chunk_text(text: str, chunk_size: int, overlap_size: int = 0) -> list:
    """
    Split text into chunks of specified size with optional overlap.

    Args:
        text: The text to chunk
        chunk_size: Size of each chunk
        overlap_size: Size of overlap between chunks

    Returns:
        List of text chunks
    """
    if not text:
        return []

    chunks = []
    start_idx = 0

    while start_idx < len(text):
        end_idx = start_idx + chunk_size

        # If we're near the end, make sure we include the remainder
        if end_idx >= len(text):
            end_idx = len(text)
        else:
            # Try to break at sentence boundaries
            # Look for sentence-ending punctuation within the last 100 characters
            chunk_content = text[start_idx:end_idx]
            sentence_break_found = False

            # Look for sentence boundaries near the end of the chunk
            for i in range(len(chunk_content) - 1, len(chunk_content) - 100, -1):
                if i > 0 and chunk_content[i] in '.!?':
                    # Found a sentence boundary, break after the punctuation
                    end_idx = start_idx + i + 1
                    sentence_break_found = True
                    break

            # If no sentence boundary found, look for paragraph boundaries
            if not sentence_break_found:
                for i in range(len(chunk_content) - 1, len(chunk_content) - 100, -1):
                    if i > 0 and chunk_content[i] == '\n' and chunk_content[i-1] == '\n':
                        # Found a paragraph boundary
                        end_idx = start_idx + i + 1
                        sentence_break_found = True
                        break

        # Extract the chunk content
        chunk_text = text[start_idx:end_idx]
        chunks.append(chunk_text)

        # Move to the next position, accounting for overlap
        start_idx = end_idx - (overlap_size if end_idx < len(text) else 0)

    return chunks