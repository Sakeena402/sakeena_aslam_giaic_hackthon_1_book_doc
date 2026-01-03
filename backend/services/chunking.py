"""
Chunking service for the Modular Retrieval System.

This module handles content segmentation and overlap logic.
"""

from typing import List
from backend.models.dataclasses import ContentPage, ContentChunk, Heading, ContentChunkState
from backend.utils.helpers import chunk_text


class ContentChunker:
    """Class to handle content chunking for semantic retrieval."""

    def __init__(self, chunk_size: int = 500, overlap_size: int = 100):
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

        # Log the number of chunks created to verify proper chunking
        from backend.utils.helpers import setup_logging
        logger = setup_logging()
        logger.info(f"Created {len(chunks)} chunks from content page '{content_page.title}' (URL: {content_page.url}). Content length: {len(content_page.content)} characters")

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
        """Create semantically-meaningful chunks with proper size constraints."""
        # Using the enhanced helper function to split text into semantic chunks
        # Use min 300, max 1000 characters as specified in requirements
        chunk_texts = chunk_text(content, min_chunk_size=300, max_chunk_size=1000, overlap_size=50)

        chunks = []
        current_pos = 0

        for i, chunk_content in enumerate(chunk_texts):
            # Calculate actual positions in the original content
            start_pos = content.find(chunk_content, current_pos)
            if start_pos == -1:  # If exact match not found, use approximation
                start_pos = current_pos
            end_pos = start_pos + len(chunk_content)
            current_pos = end_pos  # Move to the end of this chunk for next search

            # Create ContentChunk with stripped content to avoid empty chunks
            chunk_content_stripped = chunk_content.strip()
            if not chunk_content_stripped:  # Skip empty chunks
                continue

            # Verify the chunk meets minimum size requirements
            if len(chunk_content_stripped) < 300:
                # If this is the last chunk and it's too small, consider merging with previous
                if i == len(chunk_texts) - 1 and chunks:
                    # Try to merge with the previous chunk if it doesn't exceed max size
                    prev_chunk = chunks[-1]
                    combined_content = prev_chunk.content + "\n\n" + chunk_content_stripped
                    if len(combined_content) <= 1000:
                        # Merge with previous chunk
                        chunks[-1].content = combined_content
                        chunks[-1].original_end_pos = end_pos
                        continue  # Skip creating a new chunk
                elif len(chunk_content_stripped) < 50:
                    # Skip chunks that are too small (less than 50 chars)
                    continue

            chunk = ContentChunk(
                id=f"{source_url}#chunk-{i}",
                content=chunk_content_stripped,
                source_page_url=source_url,
                chunk_index=i,
                original_start_pos=start_pos,
                original_end_pos=end_pos,
                state=ContentChunkState.CHUNKED  # Using the imported enum
            )

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

    def _find_relevant_heading(self, chunk: ContentChunk, headings: List[Heading]) -> Heading:
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