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


def chunk_text(text: str, min_chunk_size: int = 300, max_chunk_size: int = 1000, overlap_size: int = 50) -> list:
    """
    Split text into semantic chunks with specified size constraints.

    Args:
        text: The text to chunk
        min_chunk_size: Minimum size of each chunk (300-400 chars)
        max_chunk_size: Maximum size of each chunk (800-1000 chars)
        overlap_size: Size of overlap between chunks

    Returns:
        List of text chunks
    """
    if not text:
        return []

    chunks = []

    # Split text into logical paragraphs/sections first
    paragraphs = text.split('\n\n')

    current_chunk = ""
    for paragraph in paragraphs:
        paragraph = paragraph.strip()
        if not paragraph:
            continue

        # Check if adding this paragraph would exceed max size
        potential_chunk = current_chunk + ('\n\n' if current_chunk else '') + paragraph

        if len(potential_chunk) <= max_chunk_size:
            # Safe to add to current chunk
            current_chunk = potential_chunk
        else:
            # Check if current chunk is substantial enough to be saved
            if len(current_chunk) >= min_chunk_size:
                # Save the current chunk and start a new one with this paragraph
                chunks.append(current_chunk)
                current_chunk = paragraph
            else:
                # Current chunk is too small, but adding paragraph would exceed max
                # Try to split the paragraph if it's too large
                if len(paragraph) > max_chunk_size:
                    # Split the large paragraph into smaller chunks
                    sub_chunks = _split_large_paragraph(paragraph, min_chunk_size, max_chunk_size)
                    if current_chunk:
                        # Add current chunk before processing sub-chunks
                        chunks.append(current_chunk)
                        current_chunk = ""

                    # Add sub-chunks (except the last one) to main chunks list
                    if len(sub_chunks) > 1:
                        chunks.extend(sub_chunks[:-1])
                        # Keep the last sub-chunk as the current chunk
                        current_chunk = sub_chunks[-1]
                    else:
                        # Only one sub-chunk, add it to current
                        current_chunk = sub_chunks[0] if sub_chunks else ""
                else:
                    # Current chunk is too small but paragraph is within limits
                    # Combine them into a new chunk
                    current_chunk = potential_chunk

    # Add the final chunk if it's substantial
    if current_chunk and len(current_chunk) >= min_chunk_size:
        chunks.append(current_chunk)
    elif current_chunk and chunks:
        # If final chunk is too small, append it to the last chunk if it exists
        last_chunk = chunks[-1]
        if len(last_chunk) + len(current_chunk) <= max_chunk_size:
            chunks[-1] = last_chunk + ('\n\n' + current_chunk if current_chunk.strip() else current_chunk)
        elif len(current_chunk) >= 50:  # If it's at least somewhat substantial, add as separate chunk
            chunks.append(current_chunk)
    elif current_chunk and len(current_chunk) >= 50:
        # If there's no previous chunk but current one is substantial, add it
        chunks.append(current_chunk)

    return chunks


def _split_large_paragraph(paragraph: str, min_chunk_size: int, max_chunk_size: int) -> list:
    """
    Split a large paragraph into smaller chunks based on sentences or semantic boundaries.

    Args:
        paragraph: The large paragraph to split
        min_chunk_size: Minimum chunk size
        max_chunk_size: Maximum chunk size

    Returns:
        List of sub-chunks
    """
    if len(paragraph) <= max_chunk_size:
        return [paragraph]

    sub_chunks = []
    sentences = []

    # Split into sentences
    import re
    # Split by sentence endings, keeping the punctuation
    sentence_parts = re.split(r'([.!?]+)', paragraph)

    # Reconstruct sentences with their punctuation
    i = 0
    while i < len(sentence_parts):
        if i + 1 < len(sentence_parts) and re.match(r'[.!?]+', sentence_parts[i + 1]):
            # This is a sentence followed by punctuation
            sentence = sentence_parts[i] + sentence_parts[i + 1]
            sentences.append(sentence.strip())
            i += 2
        else:
            # This might be a part without punctuation
            sentences.append(sentence_parts[i].strip())
            i += 1

    # Clean up empty sentences
    sentences = [s for s in sentences if s.strip()]

    current_sub_chunk = ""
    for sentence in sentences:
        potential_sub_chunk = current_sub_chunk + (' ' if current_sub_chunk else '') + sentence

        if len(potential_sub_chunk) <= max_chunk_size:
            current_sub_chunk = potential_sub_chunk
        else:
            # Check if current sub-chunk is substantial
            if len(current_sub_chunk) >= min_chunk_size:
                # Save current and start new
                sub_chunks.append(current_sub_chunk)
                current_sub_chunk = sentence
            else:
                # Current sub-chunk is too small, try to split sentence
                if len(sentence) > max_chunk_size:
                    # Sentence is too long, split it by words
                    words = sentence.split()
                    temp_chunk = current_sub_chunk
                    temp_sentence = ""

                    for word in words:
                        potential = temp_sentence + (' ' if temp_sentence else '') + word
                        if len(temp_chunk + (' ' if temp_chunk else '') + potential) <= max_chunk_size:
                            temp_sentence += (' ' if temp_sentence else '') + word
                        else:
                            if temp_sentence:
                                if temp_chunk:
                                    sub_chunks.append(temp_chunk + (' ' if temp_chunk else '') + temp_sentence)
                                else:
                                    sub_chunks.append(temp_sentence)
                                temp_chunk = ""
                                temp_sentence = word
                            else:
                                # If even a single word is too much, force split
                                if len(word) > max_chunk_size:
                                    sub_chunks.extend(_force_split_long_word(word, max_chunk_size))
                                else:
                                    temp_sentence = word
                    if temp_sentence:
                        if temp_chunk:
                            sub_chunks.append(temp_chunk + (' ' if temp_chunk else '') + temp_sentence)
                        else:
                            sub_chunks.append(temp_sentence)
                else:
                    # Start a new chunk with this sentence
                    if current_sub_chunk:
                        sub_chunks.append(current_sub_chunk)
                    current_sub_chunk = sentence

    # Add the final sub-chunk if it exists
    if current_sub_chunk:
        sub_chunks.append(current_sub_chunk)

    return sub_chunks


def _force_split_long_word(word: str, max_chunk_size: int) -> list:
    """
    Force split a very long word into smaller parts.

    Args:
        word: The long word to split
        max_chunk_size: Maximum size for each part

    Returns:
        List of word parts
    """
    parts = []
    for i in range(0, len(word), max_chunk_size):
        parts.append(word[i:i + max_chunk_size])
    return parts