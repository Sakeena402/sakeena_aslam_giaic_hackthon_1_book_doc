# """
# Embedding service for the Modular Retrieval System.

# This module handles embedding generation and validation.
# """

# import cohere
# import time
# from typing import List
# from backend.models.dataclasses import ContentChunk, EmbeddingVector, EmbeddingVectorState


# class EmbeddingGenerator:
#     """Class to handle embedding generation using Cohere."""

#     def __init__(self, api_key: str, model_name: str = "embed-multilingual-v3.0"):
#         self.client = cohere.Client(api_key)
#         self.model_name = model_name

#     def generate_embeddings(self, chunks: List[ContentChunk]) -> List[EmbeddingVector]:
#         """
#         Generate embeddings for content chunks using Cohere with rate limiting and retry logic.

#         Args:
#             chunks: List of ContentChunk objects

#         Returns:
#             List of EmbeddingVector objects
#         """
#         if not chunks:
#             from backend.utils.helpers import setup_logging
#             logger = setup_logging()
#             logger.warning("No chunks provided for embedding generation")
#             return []

#         # Extract text content from chunks
#         texts = [chunk.content for chunk in chunks]

#         # Retry mechanism with rate limiting
#         max_retries = 3
#         base_delay = 1.5  # 1.5 seconds as requested

#         for attempt in range(max_retries):
#             try:
#                 # Generate embeddings using Cohere
#                 response = self.client.embed(
#                     texts=texts,
#                     model=self.model_name,
#                     input_type="search_document"
#                 )

#                 embeddings = []
#                 for i, embedding_values in enumerate(response.embeddings):
#                     embedding_vector = EmbeddingVector(
#                         chunk_id=chunks[i].id,
#                         vector=embedding_values,
#                         model_name=self.model_name,
#                         model_version="3.0",  # This might need to be updated based on actual model
#                         state=EmbeddingVectorState.EMBEDDED  # Using the imported enum
#                     )
#                     embeddings.append(embedding_vector)

#                 from backend.utils.helpers import setup_logging
#                 logger = setup_logging()
#                 logger.info(f"Generated embeddings for {len(embeddings)} chunks")
#                 return embeddings

#             except Exception as e:
#                 from backend.utils.helpers import setup_logging
#                 logger = setup_logging()

#                 # Check if it's a rate limit error
#                 if "429" in str(e) or "TooManyRequests" in str(e) or "rate limit" in str(e).lower():
#                     logger.warning(f"Rate limit exceeded on attempt {attempt + 1}, waiting {base_delay}s before retry...")

#                     time.sleep(base_delay)
#                     base_delay *= 2  # Exponential backoff
#                 elif "Connection" in str(e) or "connection" in str(e).lower() or "network" in str(e).lower():
#                     logger.warning(f"Network error on attempt {attempt + 1}, waiting {base_delay}s before retry...")
#                     time.sleep(base_delay)
#                     base_delay *= 2  # Exponential backoff
#                 else:
#                     logger.error(f"Failed to generate embeddings: {str(e)}")
#                     if attempt == max_retries - 1:  # Last attempt
#                         # Return empty list or failed embeddings as appropriate
#                         return []

#         # If all retries failed, return empty list
#         logger.error("All retry attempts failed for embedding generation")
#         return []




"""
Embedding service for the Modular Retrieval System.

This module handles embedding generation and validation with batching and rate-limit handling.
"""

import cohere
import time
from typing import List
from ..models.dataclasses import ContentChunk, EmbeddingVector, EmbeddingVectorState
from ..utils.helpers import setup_logging

logger = setup_logging()


class EmbeddingGenerator:
    """Class to handle embedding generation using Cohere."""

    def __init__(self, api_key: str, model_name: str = "embed-multilingual-v3.0"):
        self.client = cohere.Client(api_key)
        self.model_name = model_name

    def generate_embeddings(
        self, chunks: List[ContentChunk], batch_size: int = 1  # Reduce batch size to 1 for better rate limiting control
    ) -> List[EmbeddingVector]:
        """
        Generate embeddings for content chunks using Cohere with proper rate limiting and retry logic.

        Args:
            chunks: List of ContentChunk objects
            batch_size: Number of chunks to send per request (reduced to 1 for better rate limiting)

        Returns:
            List of EmbeddingVector objects
        """
        if not chunks:
            logger.warning("No chunks provided for embedding generation")
            return []

        embeddings_result = []

        # Process chunks one at a time to ensure proper rate limiting
        for i, chunk in enumerate(chunks):
            # Skip empty or very short chunks
            if not chunk.content or len(chunk.content.strip()) < 10:
                logger.warning(f"Skipping chunk {chunk.id} with insufficient content for embedding")
                continue

            max_retries = 5
            delay = 1.5  # Start with 1.5 seconds as requested

            for attempt in range(max_retries):
                try:
                    response = self.client.embed(
                        texts=[chunk.content],
                        model=self.model_name,
                        input_type="search_document",
                    )

                    if response.embeddings and len(response.embeddings) > 0:
                        emb_vector = EmbeddingVector(
                            chunk_id=chunk.id,
                            vector=response.embeddings[0],  # Take first (and only) embedding
                            model_name=self.model_name,
                            model_version="3.0",
                            state=EmbeddingVectorState.EMBEDDED,
                        )
                        embeddings_result.append(emb_vector)

                        logger.info(f"Generated embedding for chunk {chunk.id} (chunk {i+1}/{len(chunks)})")
                        time.sleep(1.5)  # 1.5 second delay between requests as requested
                    else:
                        logger.warning(f"No embeddings returned for chunk {chunk.id}")

                    break  # exit retry loop if successful

                except Exception as e:
                    err_str = str(e).lower()
                    if "429" in err_str or "too many requests" in err_str or "rate limit" in err_str:
                        logger.warning(
                            f"Rate limit exceeded on chunk {chunk.id}, attempt {attempt+1}, waiting {delay}s..."
                        )
                    elif "connection" in err_str or "network" in err_str or "timeout" in err_str:
                        logger.warning(
                            f"Network/timeout error on chunk {chunk.id}, attempt {attempt+1}, waiting {delay}s..."
                        )
                    else:
                        logger.error(f"Failed to generate embedding for chunk {chunk.id}: {e}")
                        if attempt == max_retries - 1:  # Last attempt
                            logger.error(f"All {max_retries} attempts failed for chunk {chunk.id}")
                            # Continue to next chunk instead of stopping the entire process
                            break

                    time.sleep(delay)
                    delay *= 2  # exponential backoff

        logger.info(f"Generated total embeddings: {len(embeddings_result)} out of {len(chunks)} attempted")
        return embeddings_result

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a given text string with proper rate limiting.

        Args:
            text: Input text to generate embedding for

        Returns:
            List of float values representing the embedding vector
        """
        if not text or not text.strip():
            logger.warning("Empty text provided for single embedding generation")
            return []

        # Skip very short text that doesn't make sense to embed
        if len(text.strip()) < 10:
            logger.warning(f"Text too short ({len(text)} chars) to embed: '{text[:50]}...'")
            return []

        max_retries = 5  # Increased max retries
        delay = 1.5  # Start with 1.5 seconds as requested

        for attempt in range(max_retries):
            try:
                response = self.client.embed(
                    texts=[text],
                    model=self.model_name,
                    input_type="search_query"  # Using search_query for query embeddings
                )

                if response.embeddings and len(response.embeddings) > 0:
                    logger.info(f"Generated single embedding for text of length {len(text)}")
                    # Add rate limiting delay after successful request
                    time.sleep(1.5)  # 1.5-2 second delay as requested
                    return response.embeddings[0]  # Return the first embedding as a list of floats
                else:
                    logger.warning("No embeddings returned from API")
                    return []

            except Exception as e:
                err_str = str(e).lower()
                if "429" in err_str or "too many requests" in err_str or "rate limit" in err_str:
                    logger.warning(f"Rate limit exceeded on single embedding attempt {attempt+1}, waiting {delay}s...")
                    time.sleep(delay)
                    delay *= 2  # Exponential backoff
                elif "connection" in err_str or "network" in err_str or "timeout" in err_str:
                    logger.warning(f"Network/timeout error on single embedding attempt {attempt+1}, waiting {delay}s...")
                    time.sleep(delay)
                    delay *= 2  # Exponential backoff
                else:
                    logger.error(f"Failed to generate single embedding: {e}")
                    if attempt == max_retries - 1:  # Last attempt
                        logger.error(f"All {max_retries} attempts failed for single embedding")
                        raise e
                    else:
                        # Wait before retrying for other errors too
                        time.sleep(delay)
                        delay *= 2
        return []
