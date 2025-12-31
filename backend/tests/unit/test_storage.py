"""
Unit tests for the storage module.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
from backend.services.storage import VectorStorage
from backend.models.dataclasses import EmbeddingVector, ContentChunk


class TestVectorStorage:
    """Unit tests for VectorStorage class."""

    @pytest.fixture
    def storage(self):
        """Fixture to create a VectorStorage instance."""
        with patch('backend.services.storage.QdrantClient') as mock_client:
            storage = VectorStorage(url="http://localhost:6333", api_key="fake-api-key")
            return storage

    def test_store_embeddings_basic(self, storage):
        """Test basic embedding storage functionality."""
        embedding_vectors = [
            EmbeddingVector(
                chunk_id="test-chunk-1",
                vector=[0.1, 0.2, 0.3],
                model_name="test-model",
                model_version="1.0"
            )
        ]

        content_chunks = [
            ContentChunk(
                id="test-chunk-1",
                content="Test content for storage.",
                source_page_url="https://example.com/test"
            )
        ]

        # Mock the client upsert method
        with patch.object(storage.client, 'upsert') as mock_upsert:
            stored_vectors = storage.store_embeddings(embedding_vectors, content_chunks)

        assert len(stored_vectors) == len(embedding_vectors)
        for stored in stored_vectors:
            assert stored.vector == embedding_vectors[0].vector
            assert stored.url == content_chunks[0].source_page_url

    def test_store_embeddings_no_matching_chunks(self, storage):
        """Test storing embeddings when no matching content chunks are found."""
        embedding_vectors = [
            EmbeddingVector(
                chunk_id="non-existent-chunk",
                vector=[0.1, 0.2, 0.3],
                model_name="test-model",
                model_version="1.0"
            )
        ]

        content_chunks = [
            ContentChunk(
                id="different-chunk-id",
                content="Different content.",
                source_page_url="https://example.com/test"
            )
        ]

        # Mock the client upsert method
        with patch.object(storage.client, 'upsert') as mock_upsert:
            stored_vectors = storage.store_embeddings(embedding_vectors, content_chunks)

        # Should return empty list since no matching chunks were found
        assert len(stored_vectors) == 0

    def test_store_embeddings_empty_list(self, storage):
        """Test storing embeddings with empty list."""
        stored_vectors = storage.store_embeddings([], [])
        assert len(stored_vectors) == 0