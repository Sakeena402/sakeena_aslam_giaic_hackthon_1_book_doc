"""
Unit tests for the embedding module.
"""

import pytest
from unittest.mock import Mock, patch
from backend.services.embedding import EmbeddingGenerator
from backend.models.dataclasses import ContentChunk


class TestEmbeddingGenerator:
    """Unit tests for EmbeddingGenerator class."""

    @pytest.fixture
    def generator(self):
        """Fixture to create an EmbeddingGenerator instance."""
        with patch('backend.services.embedding.cohere'):
            return EmbeddingGenerator(api_key="fake-api-key")

    def test_generate_embeddings_basic(self, generator):
        """Test basic embedding generation functionality."""
        chunks = [
            ContentChunk(
                id="test-chunk-1",
                content="This is a test content for embedding.",
                source_page_url="https://example.com/test"
            ),
            ContentChunk(
                id="test-chunk-2",
                content="Another piece of content for embedding.",
                source_page_url="https://example.com/test2"
            )
        ]

        # Mock the cohere client embed method
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]

        with patch.object(generator.client, 'embed', return_value=mock_response):
            embeddings = generator.generate_embeddings(chunks)

        assert len(embeddings) == len(chunks)
        for i, embedding in enumerate(embeddings):
            assert embedding.chunk_id == chunks[i].id
            assert len(embedding.vector) == 3  # Mock vector size

    def test_generate_embeddings_empty_list(self, generator):
        """Test embedding generation with empty chunk list."""
        embeddings = generator.generate_embeddings([])
        assert len(embeddings) == 0

    def test_generate_embeddings_with_error(self, generator):
        """Test embedding generation with API error."""
        chunks = [
            ContentChunk(
                id="test-chunk-1",
                content="This is a test content for embedding.",
                source_page_url="https://example.com/test"
            )
        ]

        # Mock the cohere client to raise an exception
        with patch.object(generator.client, 'embed', side_effect=Exception("API Error")):
            embeddings = generator.generate_embeddings(chunks)

        assert len(embeddings) == 0