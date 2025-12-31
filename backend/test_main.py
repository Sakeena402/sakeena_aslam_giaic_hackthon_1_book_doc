"""
Simple test file to verify the main functionality of the content extraction system.
"""
import os
from main import ContentPage, Heading, ContentChunk, EmbeddingVector, StoredVector, Config

def test_data_classes():
    """Test that the data classes work correctly."""
    print("Testing data classes...")

    # Test ContentPage
    try:
        page = ContentPage(
            url="https://example.com/page1",
            title="Test Page",
            content="This is some test content."
        )
        print(f"SUCCESS: ContentPage created: {page.title}")
    except Exception as e:
        print(f"ERROR: ContentPage creation failed: {e}")

    # Test Heading
    try:
        heading = Heading(level=1, text="Main Title", position=0)
        print(f"SUCCESS: Heading created: {heading.text}")
    except Exception as e:
        print(f"ERROR: Heading creation failed: {e}")

    # Test ContentChunk
    try:
        chunk = ContentChunk(
            id="test-chunk-1",
            content="This is a test chunk of content.",
            source_page_url="https://example.com/page1",
            chunk_index=0
        )
        print(f"SUCCESS: ContentChunk created: {chunk.id}")
    except Exception as e:
        print(f"ERROR: ContentChunk creation failed: {e}")

    # Test EmbeddingVector
    try:
        embedding = EmbeddingVector(
            chunk_id="test-chunk-1",
            vector=[0.1, 0.2, 0.3],
            model_name="test-model",
            model_version="1.0"
        )
        print(f"SUCCESS: EmbeddingVector created with {len(embedding.vector)} dimensions")
    except Exception as e:
        print(f"ERROR: EmbeddingVector creation failed: {e}")

    # Test StoredVector
    try:
        from datetime import datetime
        stored = StoredVector(
            id="test-stored-1",
            vector=[0.1, 0.2, 0.3],
            url="https://example.com/page1",
            title="Test Page",
            chunk_index=0,
            content_preview="This is a test chunk of content.",
            source_created_at=datetime.now()
        )
        print(f"SUCCESS: StoredVector created: {stored.id}")
    except Exception as e:
        print(f"ERROR: StoredVector creation failed: {e}")

    # Test Config
    try:
        # Temporarily set environment variables for testing
        os.environ['COHERE_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test-qdrant.com'
        os.environ['QDRANT_API_KEY'] = 'test-api-key'
        os.environ['BOOK_BASE_URL'] = 'https://test-book.com'

        config = Config()
        print(f"SUCCESS: Config created with base URL: {config.book_base_url}")
    except Exception as e:
        print(f"ERROR: Config creation failed: {e}")
    finally:
        # Clean up test environment variables
        for key in ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'BOOK_BASE_URL']:
            if key in os.environ:
                del os.environ[key]

    print("Data class tests completed.")


if __name__ == "__main__":
    test_data_classes()