"""
Chunking service for the Modular Retrieval System.

This module handles content segmentation and overlap logic.
"""

from typing import List
from backend.models.dataclasses import ContentPage, ContentChunk, Heading, ContentChunkState
from backend.utils.helpers import chunk_text


class ContentChunker:
    """Class to handle content chunking for semantic retrieval."""

    def __init__(self, chunk_size: int = 1000, overlap_size: int = 200):
        self.chunk_size = chunk_size
        self.overlap_size = overlap_size

    def chunk_content_page(self, content_page: ContentPage) -> List[ContentChunk]:
        """
        Split content from a ContentPage into appropriately sized chunks with overlap.

        Args:
            content_page: The ContentPage to chunk

        Returns:
            List of ContentChunk objects
        """
        if not content_page.content:
            from backend.utils.helpers import setup_logging
            logger = setup_logging()
            logger.warning(f"No content to chunk for URL: {content_page.url}")
            return []

        # Split content into chunks
        chunks = self._create_chunks(content_page.content, content_page.url)

        # Add heading context to chunks if available
        if content_page.headings:
            chunks = self._add_heading_context(chunks, content_page.headings)

        # Set chunk metadata
        for i, chunk in enumerate(chunks):
            chunk.chunk_index = i
            chunk.id = f"{content_page.url}#chunk-{i}"
            chunk.state = ContentChunkState.CHUNKED  # Using the imported enum

        from backend.utils.helpers import setup_logging
        logger = setup_logging()
        logger.info(f"Created {len(chunks)} chunks from content page: {content_page.url}")
        return chunks

    def _create_chunks(self, content: str, source_url: str) -> List[ContentChunk]:
        """Create chunks with overlap from content."""
        # Using the helper function to split text into chunks
        chunk_texts = chunk_text(content, self.chunk_size, self.overlap_size)

        chunks = []
        for i, chunk_content in enumerate(chunk_texts):
            # Create ContentChunk
            chunk = ContentChunk(
                id=f"{source_url}#chunk-{i}",
                content=chunk_content,
                source_page_url=source_url,
                chunk_index=i,
                original_start_pos=i * (self.chunk_size - self.overlap_size),  # Approximate position
                original_end_pos=min((i + 1) * (self.chunk_size - self.overlap_size) + self.overlap_size, len(content)),
                state=ContentChunkState.CHUNKED  # Using the imported enum
            )

            # Add overlap with next chunk if it exists
            if i < len(chunk_texts) - 1:
                # Calculate overlap with next chunk
                next_chunk_start = chunk_texts[i + 1][:self.overlap_size]
                chunk.overlap_with_next = next_chunk_start

            chunks.append(chunk)

        return chunks

    def _add_heading_context(self, chunks: List[ContentChunk], headings: List[Heading]) -> List[ContentChunk]:
        """Add heading context to chunks based on their position in the original content."""
        for chunk in chunks:
            # Find the most relevant heading for this chunk based on position
            relevant_heading = self._find_relevant_heading(chunk, headings)
            if relevant_heading:
                chunk.heading_context = relevant_heading.text

        return chunks

    def _find_relevant_heading(self, chunk: ContentChunk, headings: List[Heading]) -> ContentChunk:
        """Find the most relevant heading for a chunk based on position."""
        # Find the heading that appears before this chunk and is closest
        relevant_heading = None
        closest_position = -1

        for heading in headings:
            # Only consider headings that appear before this chunk
            if heading.position < chunk.original_start_pos and heading.position > closest_position:
                relevant_heading = heading
                closest_position = heading.position

        return relevant_heading