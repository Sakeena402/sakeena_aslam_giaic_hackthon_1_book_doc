"""
Unit tests for the retrieval module.
"""

import pytest
from unittest.mock import Mock, patch
from backend.services.retrieval import QdrantRetriever, simple_retrieve_chunks
from backend.config.settings import Config
from backend.models.dataclasses import RetrievedChunk


class TestQdrantRetriever:
    """Unit tests for QdrantRetriever class."""

    @pytest.fixture
    def config(self):
        """Fixture to create a mock configuration."""
        config = Mock(spec=Config)
        config.qdrant_url = "http://localhost:6333"
        config.qdrant_api_key = "fake-api-key"
        config.qdrant_collection_name = "test-collection"
        config.request_timeout = 30
        return config

    @pytest.fixture
    def retriever(self, config):
        """Fixture to create a QdrantRetriever instance."""
        with patch('backend.services.retrieval.QdrantClient') as mock_client:
            return QdrantRetriever(config)

    def test_search_similar(self, retriever, config):
        """Test similarity search functionality."""
        query_embedding = [0.1, 0.2, 0.3]

        # Create a mock search result
        mock_result = Mock()
        mock_result.id = "test-id"
        mock_result.score = 0.8
        mock_result.payload = {
            "url": "https://example.com/test",
            "title": "Test Page",
            "chunk_index": 1,
            "content_preview": "Test content preview...",
            "heading_context": "Test Heading"
        }

        with patch.object(retriever.client, 'search', return_value=[mock_result]):
            results = retriever.search_similar(query_embedding, top_k=5, similarity_threshold=0.5)

        assert len(results) == 1
        assert isinstance(results[0], RetrievedChunk)
        assert results[0].similarity_score == 0.8
        assert results[0].metadata["url"] == "https://example.com/test"

    def test_search_with_filters(self, retriever, config):
        """Test similarity search with filters functionality."""
        query_embedding = [0.1, 0.2, 0.3]

        # Create a mock search result
        mock_result = Mock()
        mock_result.id = "test-id"
        mock_result.score = 0.7
        mock_result.payload = {
            "url": "https://example.com/test",
            "title": "Test Page",
            "chunk_index": 1,
            "content_preview": "Test content preview...",
            "heading_context": "Test Heading"
        }

        mock_filter = Mock()

        with patch.object(retriever.client, 'search', return_value=[mock_result]):
            results = retriever.search_with_filters(query_embedding, mock_filter, top_k=5, similarity_threshold=0.5)

        assert len(results) == 1
        assert isinstance(results[0], RetrievedChunk)
        assert results[0].similarity_score == 0.7


def test_simple_retrieve_chunks():
    """Test the simple retrieval function."""
    config = Mock()
    config.cohere_api_key = "fake-api-key"
    config.retrieval_model_name = "embed-multilingual-v3.0"
    config.qdrant_url = "http://localhost:6333"
    config.qdrant_api_key = "fake-api-key"
    config.qdrant_collection_name = "test-collection"

    query_text = "test query"
    query_embedding = [0.1, 0.2, 0.3]

    # Mock the Cohere client
    with patch('backend.services.retrieval.cohere') as mock_cohere_module:
        mock_cohere = Mock()
        mock_cohere_module.Client.return_value = mock_cohere

        mock_embed_response = Mock()
        mock_embed_response.embeddings = [query_embedding]
        mock_cohere.embed.return_value = mock_embed_response

        # Mock the QdrantRetriever
        with patch('backend.services.retrieval.QdrantRetriever') as mock_retriever_class:
            mock_retriever = Mock()
            mock_retriever_class.return_value = mock_retriever

            mock_chunk = RetrievedChunk(
                id="test-id",
                content="Test content",
                similarity_score=0.8,
                metadata={"url": "https://example.com/test", "title": "Test", "chunk_index": 1, "content_preview": "Preview"}
            )

            mock_retriever.search_similar.return_value = [mock_chunk]

            results = simple_retrieve_chunks(query_text, config)

    assert len(results) == 1
    assert isinstance(results[0], RetrievedChunk)
    assert results[0].content == "Test content"
    assert results[0].similarity_score == 0.8