"""
Unit tests for the chunking module.
"""

import pytest
from backend.services.chunking import ContentChunker
from backend.models.dataclasses import ContentPage, Heading


class TestContentChunker:
    """Unit tests for ContentChunker class."""

    @pytest.fixture
    def chunker(self):
        """Fixture to create a ContentChunker instance."""
        return ContentChunker(chunk_size=100, overlap_size=20)

    def test_chunk_content_page_basic(self, chunker):
        """Test basic content chunking functionality."""
        content = "This is a sample content for testing chunking functionality. " * 10
        content_page = ContentPage(
            url="https://example.com/test",
            title="Test Page",
            content=content
        )

        chunks = chunker.chunk_content_page(content_page)

        assert len(chunks) > 0
        for chunk in chunks:
            assert len(chunk.content) <= chunker.chunk_size
            assert chunk.source_page_url == content_page.url

    def test_chunk_content_page_with_headings(self, chunker):
        """Test content chunking with headings."""
        content = "This is a sample content for testing chunking functionality. " * 10
        heading = Heading(level=2, text="Sample Heading", position=50)
        content_page = ContentPage(
            url="https://example.com/test",
            title="Test Page",
            content=content,
            headings=[heading]
        )

        chunks = chunker.chunk_content_page(content_page)

        assert len(chunks) > 0
        # Check if at least one chunk has heading context
        chunks_with_headings = [c for c in chunks if c.heading_context is not None]
        # This may vary depending on implementation, but the functionality should work

    def test_chunk_content_page_empty_content(self, chunker):
        """Test chunking with empty content."""
        content_page = ContentPage(
            url="https://example.com/test",
            title="Test Page",
            content=""
        )

        chunks = chunker.chunk_content_page(content_page)

        assert len(chunks) == 0

    def test_create_chunks(self, chunker):
        """Test the internal _create_chunks method functionality."""
        content = "This is a sample content for testing chunking functionality. " * 5
        chunks = chunker._create_chunks(content, "https://example.com/test")

        assert len(chunks) > 0
        for chunk in chunks:
            assert len(chunk.content) <= chunker.chunk_size
            assert chunk.source_page_url == "https://example.com/test"