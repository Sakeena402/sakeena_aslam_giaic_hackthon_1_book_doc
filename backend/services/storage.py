"""
Storage service for the Modular Retrieval System.

This module handles Qdrant operations and vector storage.
"""

import uuid
from datetime import datetime
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
from qdrant_client import models
from backend.models.dataclasses import EmbeddingVector, ContentChunk, StoredVector, StoredVectorState


class VectorStorage:
    """Class to handle vector storage in Qdrant."""

    def __init__(self, url: str, api_key: str, collection_name: str = "book_content"):
        self.client = QdrantClient(url=url, api_key=api_key, prefer_grpc=False)
        self.collection_name = collection_name
        self.vector_size = 1024  # For Cohere embed-multilingual-v3.0
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        """Ensure the Qdrant collection exists with proper configuration."""
        try:
            # Try to get collection info to check if it exists
            self.client.get_collection(self.collection_name)
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(size=self.vector_size, distance=Distance.COSINE),
            )
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Created collection {self.collection_name} with {self.vector_size}-dimension vectors")

    def store_embeddings(self, embedding_vectors: List[EmbeddingVector], content_chunks: List[ContentChunk]) -> List[StoredVector]:
        """
        Store embedding vectors in Qdrant with metadata.

        Args:
            embedding_vectors: List of EmbeddingVector objects to store
            content_chunks: Corresponding ContentChunk objects for metadata

        Returns:
            List of StoredVector objects
        """
        if not embedding_vectors:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.warning("No embeddings provided for storage")
            return []

        # Create points for Qdrant
        points = []
        stored_vectors = []

        for i, emb_vector in enumerate(embedding_vectors):
            # Find corresponding content chunk
            chunk = next((c for c in content_chunks if c.id == emb_vector.chunk_id), None)

            if not chunk:
                from backend.utils.helpers import setup_logging
                logger = setup_logging()
                logger.warning(f"No matching content chunk found for embedding: {emb_vector.chunk_id}")
                continue

            # Prepare payload with metadata
            payload = {
                "url": chunk.source_page_url,
                "chunk_index": chunk.chunk_index,
                "content_preview": chunk.content[:200] + "..." if len(chunk.content) > 200 else chunk.content,
                "source_created_at": str(chunk.original_start_pos),  # This might need adjustment
            }

            # Add heading context if available
            if chunk.heading_context:
                payload["heading_context"] = chunk.heading_context

            # Create Qdrant point
            point_id = str(uuid.uuid4())
            point = {
                "id": point_id,
                "vector": emb_vector.vector,
                "payload": payload
            }
            points.append(point)

            # Create StoredVector object
            stored_vector = StoredVector(
                id=point_id,
                vector=emb_vector.vector,
                url=chunk.source_page_url,
                title=chunk.heading_context or chunk.source_page_url.split('/')[-1],  # Use heading or URL as title
                chunk_index=chunk.chunk_index,
                content_preview=payload["content_preview"],
                source_created_at=datetime.now(),  # This should come from the original content page
                chapter="",  # This would need to be extracted from the content structure
                section="",  # This would need to be extracted from the content structure
                collection_name=self.collection_name,
                state=StoredVectorState.STORED  # Using the imported enum
            )
            stored_vectors.append(stored_vector)

        # Upload points to Qdrant
        try:
            if points:
                # Convert points to the format Qdrant client expects
                points_list = [
                    models.PointStruct(
                        id=point["id"],
                        vector=point["vector"],
                        payload=point["payload"]
                    ) for point in points
                ]

                self.client.upsert(
                    collection_name=self.collection_name,
                    points=points_list
                )

                from backend.utils.helpers import setup_logging
                logger = setup_logging()
                logger.info(f"Successfully stored {len(points)} vectors in Qdrant collection {self.collection_name}")
            else:
                from backend.utils.helpers import setup_logging
                logger = setup_logging()
                logger.warning("No points to upload to Qdrant")
        except Exception as e:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Failed to store vectors in Qdrant: {str(e)}")
            # In a real implementation, we might want to handle this differently
            return []

        return stored_vectors