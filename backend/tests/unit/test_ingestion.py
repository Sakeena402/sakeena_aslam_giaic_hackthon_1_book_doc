"""
Unit tests for the ingestion module.
"""

import pytest
from unittest.mock import Mock, patch
from backend.services.ingestion import ContentExtractor
from backend.config.settings import Config
from backend.models.dataclasses import ContentPage


class TestContentExtractor:
    """Unit tests for ContentExtractor class."""

    @pytest.fixture
    def config(self):
        """Fixture to create a mock configuration."""
        config = Mock(spec=Config)
        config.request_timeout = 30
        config.book_base_url = "https://example.com"
        return config

    @pytest.fixture
    def extractor(self, config):
        """Fixture to create a ContentExtractor instance."""
        return ContentExtractor(config)

    def test_extract_content_from_url_success(self, extractor, config):
        """Test successful content extraction from a URL."""
        # Mock the session.get response
        mock_response = Mock()
        mock_response.text = '<html><head><title>Test Page</title></head><body><h1>Header</h1><p>Content text</p></body></html>'
        mock_response.raise_for_status.return_value = None

        with patch.object(extractor.session, 'get', return_value=mock_response):
            result = extractor.extract_content_from_url("https://example.com/test")

        assert isinstance(result, ContentPage)
        assert result.title == "Test Page"
        assert "Content text" in result.content

    def test_extract_content_from_url_failure(self, extractor, config):
        """Test content extraction failure handling."""
        with patch.object(extractor.session, 'get', side_effect=Exception("Network error")):
            result = extractor.extract_content_from_url("https://example.com/test")

        assert isinstance(result, ContentPage)
        assert result.state == ContentPage.ContentPageState.FAILED

    def test_extract_from_sitemap(self, extractor, config):
        """Test extracting content from a sitemap."""
        sitemap_xml = '''<?xml version="1.0" encoding="UTF-8"?>
        <urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">
            <url>
                <loc>https://example.com/page1</loc>
            </url>
            <url>
                <loc>https://example.com/page2</loc>
            </url>
        </urlset>'''

        mock_response = Mock()
        mock_response.text = sitemap_xml
        mock_response.raise_for_status.return_value = None

        with patch.object(extractor.session, 'get', return_value=mock_response):
            # Mock the extract_content_from_url method to avoid actual extraction
            with patch.object(extractor, 'extract_content_from_url') as mock_extract:
                mock_extract.return_value = ContentPage(
                    url="https://example.com/page1",
                    title="Page 1",
                    content="Content of page 1",
                    state=ContentPage.ContentPageState.EXTRACTED
                )

                result = extractor.extract_from_sitemap("https://example.com/sitemap.xml")

        # The method should return a list of ContentPage objects
        assert isinstance(result, list)
        # We mocked the extraction, so it should return the mocked result for each URL