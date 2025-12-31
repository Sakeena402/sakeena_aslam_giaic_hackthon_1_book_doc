"""
Integration tests for the end-to-end pipeline.
"""

import pytest
from unittest.mock import Mock, patch
from backend.config.settings import Config
from backend.models.dataclasses import ContentPage, ContentChunk, EmbeddingVector, RetrievedChunk
from backend.services.ingestion import ContentExtractor
from backend.services.chunking import ContentChunker
from backend.services.embedding import EmbeddingGenerator
from backend.services.storage import VectorStorage
from backend.services.retrieval import simple_retrieve_chunks


class TestEndToEndPipeline:
    """Integration tests for the complete pipeline."""

    @pytest.fixture
    def config(self):
        """Fixture to create a mock configuration."""
        config = Mock(spec=Config)
        config.cohere_api_key = "fake-api-key"
        config.qdrant_url = "http://localhost:6333"
        config.qdrant_api_key = "fake-api-key"
        config.qdrant_collection_name = "test-collection"
        config.request_timeout = 30
        config.chunk_size = 1000
        config.overlap_size = 200
        config.retrieval_model_name = "embed-multilingual-v3.0"
        config.book_base_url = "https://example.com"
        return config

    @pytest.mark.integration
    def test_complete_pipeline_flow(self, config):
        """Test the complete pipeline flow: ingestion -> chunking -> embedding -> storage -> retrieval."""
        # Mock content for testing
        sample_content = "This is a sample content for testing the complete pipeline. " * 10

        # Step 1: Simulate content ingestion
        content_page = ContentPage(
            url="https://example.com/test",
            title="Test Page",
            content=sample_content
        )

        # Step 2: Content chunking
        chunker = ContentChunker(chunk_size=config.chunk_size, overlap_size=config.overlap_size)
        chunks = chunker.chunk_content_page(content_page)

        assert len(chunks) > 0, "Chunks should be created from content page"

        # Step 3: Embedding generation (mocked)
        mock_embeddings = [[float(i % 100) / 100.0 for _ in range(1024)] for i in range(len(chunks))]

        embedding_vectors = []
        for i, chunk in enumerate(chunks):
            embedding_vector = EmbeddingVector(
                chunk_id=chunk.id,
                vector=mock_embeddings[i],
                model_name=config.retrieval_model_name,
                model_version="3.0"
            )
            embedding_vectors.append(embedding_vector)

        assert len(embedding_vectors) == len(chunks), "Each chunk should have a corresponding embedding"

        # Step 4: Storage (mocked)
        # We're not actually storing in Qdrant in this test, just verifying the flow
        with patch('backend.services.storage.QdrantClient') as mock_client:
            storage = VectorStorage(config.qdrant_url, config.qdrant_api_key, config.qdrant_collection_name)

            # Mock the upsert method to avoid actual storage
            with patch.object(storage.client, 'upsert'):
                stored_vectors = storage.store_embeddings(embedding_vectors, chunks)

        # Step 5: Retrieval (mocked)
        # Mock the retrieval process to simulate a query
        with patch('backend.services.retrieval.cohere') as mock_cohere:
            # Setup mock Cohere client
            mock_cohere_client = Mock()
            mock_cohere.Client.return_value = mock_cohere_client

            # Mock embedding response
            mock_embed_response = Mock()
            mock_embed_response.embeddings = [mock_embeddings[0]]  # Use first embedding for query
            mock_cohere_client.embed.return_value = mock_embed_response

            # Mock Qdrant client for retrieval
            with patch('backend.services.retrieval.QdrantClient') as mock_qdrant:
                mock_retriever_instance = Mock()
                mock_qdrant.return_value = mock_retriever_instance

                # Mock search results
                mock_search_result = Mock()
                mock_search_result.id = "test-result-id"
                mock_search_result.score = 0.8
                mock_search_result.payload = {
                    "url": "https://example.com/test",
                    "title": "Test Page",
                    "chunk_index": 0,
                    "content_preview": sample_content[:100],
                    "heading_context": "Test Heading"
                }

                mock_retriever_instance.search.return_value = [mock_search_result]

                # Perform retrieval
                query_result = simple_retrieve_chunks("test query", config)

        # Verify the retrieval result
        assert isinstance(query_result, list)
        if query_result:  # If we got results back
            assert isinstance(query_result[0], RetrievedChunk)
            assert query_result[0].similarity_score >= 0.0 and query_result[0].similarity_score <= 1.0

    @pytest.mark.integration
    def test_pipeline_with_multiple_chunks(self, config):
        """Test the pipeline with multiple content chunks."""
        # Create multiple content pages
        content_pages = [
            ContentPage(
                url="https://example.com/test1",
                title="Test Page 1",
                content="This is the first sample content for testing. " * 5
            ),
            ContentPage(
                url="https://example.com/test2",
                title="Test Page 2",
                content="This is the second sample content for testing. " * 5
            )
        ]

        # Chunk all content pages
        chunker = ContentChunker(chunk_size=config.chunk_size, overlap_size=config.overlap_size)
        all_chunks = []
        for page in content_pages:
            chunks = chunker.chunk_content_page(page)
            all_chunks.extend(chunks)

        assert len(all_chunks) > 1, "Should have multiple chunks from multiple pages"

        # Generate mock embeddings
        mock_embeddings = [[float((j + i) % 100) / 100.0 for _ in range(1024)]
                          for i, chunk in enumerate(all_chunks)]

        embedding_vectors = []
        for i, chunk in enumerate(all_chunks):
            embedding_vector = EmbeddingVector(
                chunk_id=chunk.id,
                vector=mock_embeddings[i],
                model_name=config.retrieval_model_name,
                model_version="3.0"
            )
            embedding_vectors.append(embedding_vector)

        assert len(embedding_vectors) == len(all_chunks)

        # Storage would normally happen here (mocked)

        # Verify all chunks have unique IDs
        chunk_ids = [chunk.id for chunk in all_chunks]
        assert len(set(chunk_ids)) == len(chunk_ids), "All chunks should have unique IDs"


@pytest.mark.parametrize("content_length", [10, 100, 1000])
def test_chunking_various_content_lengths(content_length):
    """Test chunking with various content lengths."""
    config = Mock()
    config.chunk_size = 100
    config.overlap_size = 10

    content = "Test content. " * content_length
    content_page = ContentPage(
        url="https://example.com/test",
        title="Test Page",
        content=content
    )

    chunker = ContentChunker(chunk_size=config.chunk_size, overlap_size=config.overlap_size)
    chunks = chunker.chunk_content_page(content_page)

    # Verify chunks were created
    assert len(chunks) > 0

    # Verify chunk sizes are within bounds
    for chunk in chunks:
        assert len(chunk.content) <= config.chunk_size