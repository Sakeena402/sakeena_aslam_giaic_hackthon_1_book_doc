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
from backend.models.dataclasses import ContentChunk, EmbeddingVector, EmbeddingVectorState
from backend.utils.helpers import setup_logging

logger = setup_logging()


class EmbeddingGenerator:
    """Class to handle embedding generation using Cohere."""

    def __init__(self, api_key: str, model_name: str = "embed-multilingual-v3.0"):
        self.client = cohere.Client(api_key)
        self.model_name = model_name

    def generate_embeddings(
        self, chunks: List[ContentChunk], batch_size: int = 5
    ) -> List[EmbeddingVector]:
        """
        Generate embeddings for content chunks using Cohere with batching, rate limiting, and retry logic.

        Args:
            chunks: List of ContentChunk objects
            batch_size: Number of chunks to send per request

        Returns:
            List of EmbeddingVector objects
        """
        if not chunks:
            logger.warning("No chunks provided for embedding generation")
            return []

        embeddings_result = []

        # Split chunks into batches
        for i in range(0, len(chunks), batch_size):
            batch = chunks[i : i + batch_size]
            texts = [chunk.content for chunk in batch]

            max_retries = 5
            delay = 1.5  # initial wait for exponential backoff

            for attempt in range(max_retries):
                try:
                    response = self.client.embed(
                        texts=texts,
                        model=self.model_name,
                        input_type="search_document",
                    )

                    for j, emb_values in enumerate(response.embeddings):
                        emb_vector = EmbeddingVector(
                            chunk_id=batch[j].id,
                            vector=emb_values,
                            model_name=self.model_name,
                            model_version="3.0",
                            state=EmbeddingVectorState.EMBEDDED,
                        )
                        embeddings_result.append(emb_vector)

                    logger.info(f"Generated embeddings for batch {i}-{i+len(batch)}")
                    break  # exit retry loop if successful

                except Exception as e:
                    err_str = str(e).lower()
                    if "429" in err_str or "too many requests" in err_str or "rate limit" in err_str:
                        logger.warning(
                            f"Rate limit exceeded on batch {i}-{i+len(batch)}, attempt {attempt+1}, waiting {delay}s..."
                        )
                    elif "connection" in err_str or "network" in err_str:
                        logger.warning(
                            f"Network error on batch {i}-{i+len(batch)}, attempt {attempt+1}, waiting {delay}s..."
                        )
                    else:
                        logger.error(f"Failed to generate embeddings for batch {i}-{i+len(batch)}: {e}")
                        break  # non-retriable error

                    time.sleep(delay)
                    delay *= 2  # exponential backoff

                if attempt == max_retries - 1:
                    logger.error(f"All retries failed for batch {i}-{i+len(batch)}")

        logger.info(f"Generated total embeddings: {len(embeddings_result)}")
        return embeddings_result
