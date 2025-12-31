"""
Unit tests for the validation module.
"""

import pytest
from backend.services.validation import (
    validate_semantic_relevance,
    validate_metadata_accuracy,
    validate_consistency,
    validate_retrieval_results,
    validate_embedding_dimensions,
    validate_chunk_integrity
)
from backend.models.dataclasses import RetrievedChunk, ValidationResult


class TestValidationFunctions:
    """Unit tests for validation functions."""

    def test_validate_semantic_relevance_basic(self):
        """Test basic semantic relevance validation."""
        query_text = "test query"

        mock_chunk = RetrievedChunk(
            id="test-id",
            content="This is a relevant content for the test query",
            similarity_score=0.8,
            metadata={"url": "https://example.com/test", "title": "Test", "chunk_index": 1, "content_preview": "Preview"}
        )

        result = validate_semantic_relevance(query_text, [mock_chunk])

        # The function should return a float between 0 and 1
        assert isinstance(result, float)
        assert 0.0 <= result <= 1.0

    def test_validate_semantic_relevance_empty_chunks(self):
        """Test semantic relevance validation with empty chunks."""
        result = validate_semantic_relevance("test query", [])
        assert result == 0.0

    def test_validate_metadata_accuracy_basic(self):
        """Test basic metadata accuracy validation."""
        mock_chunk = RetrievedChunk(
            id="test-id",
            content="Test content",
            similarity_score=0.8,
            metadata={
                "url": "https://example.com/test",
                "title": "Test Title",
                "chunk_index": 1,
                "content_preview": "Preview content"
            }
        )

        result = validate_metadata_accuracy([mock_chunk])

        assert isinstance(result, float)
        assert 0.0 <= result <= 1.0
        assert result == 1.0  # All required fields are present

    def test_validate_metadata_accuracy_missing_fields(self):
        """Test metadata accuracy validation with missing fields."""
        mock_chunk = RetrievedChunk(
            id="test-id",
            content="Test content",
            similarity_score=0.8,
            metadata={
                "url": "https://example.com/test",
                "title": "Test Title",
                # Missing chunk_index and content_preview
            }
        )

        result = validate_metadata_accuracy([mock_chunk])

        assert isinstance(result, float)
        assert 0.0 <= result <= 1.0
        assert result < 1.0  # Some required fields are missing

    def test_validate_metadata_accuracy_empty_chunks(self):
        """Test metadata accuracy validation with empty chunks."""
        result = validate_metadata_accuracy([])
        assert result == 0.0

    def test_validate_consistency(self):
        """Test consistency validation."""
        config = Mock()
        result = validate_consistency("test query", config, num_tests=2)

        # The function returns 1.0 as a placeholder in the current implementation
        assert result == 1.0

    def test_validate_retrieval_results(self):
        """Test comprehensive retrieval validation."""
        from backend.config.settings import Config

        config = Mock(spec=Config)

        mock_chunk = RetrievedChunk(
            id="test-id",
            content="Test content",
            similarity_score=0.8,
            metadata={
                "url": "https://example.com/test",
                "title": "Test Title",
                "chunk_index": 1,
                "content_preview": "Preview content"
            }
        )

        result = validate_retrieval_results("test query", [mock_chunk], config)

        assert isinstance(result, ValidationResult)
        assert result.query_text == "test query"
        assert len(result.retrieved_chunks) == 1
        assert isinstance(result.semantic_relevance_score, float)
        assert isinstance(result.metadata_accuracy, float)
        assert isinstance(result.latency_ms, float)
        assert isinstance(result.consistency_score, float)

    def test_validate_embedding_dimensions_correct(self):
        """Test embedding dimension validation with correct size."""
        embedding = [0.1] * 1024  # Cohere embed-multilingual-v3.0 has 1024 dimensions
        result = validate_embedding_dimensions(embedding)
        assert result is True

    def test_validate_embedding_dimensions_incorrect(self):
        """Test embedding dimension validation with incorrect size."""
        embedding = [0.1] * 512  # Wrong size
        result = validate_embedding_dimensions(embedding)
        assert result is False

    def test_validate_embedding_dimensions_custom_size(self):
        """Test embedding dimension validation with custom expected size."""
        embedding = [0.1] * 256
        result = validate_embedding_dimensions(embedding, expected_size=256)
        assert result is True

    def test_validate_chunk_integrity_valid(self):
        """Test chunk integrity validation with valid content."""
        content = "This is a valid content chunk."
        result = validate_chunk_integrity(content)
        assert result is True

    def test_validate_chunk_integrity_short(self):
        """Test chunk integrity validation with too short content."""
        content = "Hi"
        result = validate_chunk_integrity(content)
        assert result is False

    def test_validate_chunk_integrity_empty(self):
        """Test chunk integrity validation with empty content."""
        content = ""
        result = validate_chunk_integrity(content)
        assert result is False

    def test_validate_chunk_integrity_many_special_chars(self):
        """Test chunk integrity validation with too many special characters."""
        content = "\x00\x01\x02" * 10  # Non-printable characters
        result = validate_chunk_integrity(content)
        assert result is False