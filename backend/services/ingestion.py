"""
Ingestion service for the Modular Retrieval System.

This module handles content extraction and URL processing from Docusaurus sites.
"""

import os
import requests
from typing import List
from bs4 import BeautifulSoup
from urllib.parse import urljoin, urlparse
from ..models.dataclasses import ContentPage, Heading, ContentPageState
from ..config.settings import Config


class ContentExtractor:
    """Class to handle content extraction from Docusaurus sites."""

    def __init__(self, config: Config):
        self.config = config
        self.session = requests.Session()
        # Set a user agent to avoid being blocked
        self.session.headers.update({
            'User-Agent': 'Mozilla/5.0 (compatible; ContentExtractor/1.0; +http://example.com/bot)'
        })

    def extract_content_from_url(self, url: str) -> ContentPage:
        """
        Extract clean content from a single URL.

        Args:
            url: The URL to extract content from

        Returns:
            ContentPage object with extracted content
        """
        try:
            response = self.session.get(url, timeout=self.config.request_timeout)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'html.parser')

            # Extract title
            title = self._extract_title(soup)

            # Extract content, filtering out navigation elements
            content, headings = self._extract_content_and_headings(soup)

            # Create and return ContentPage
            content_page = ContentPage(
                url=url,
                title=title,
                content=content,
                headings=headings,
                state=ContentPageState.EXTRACTED  # Using the imported enum
            )

            # Import logging here to avoid circular dependencies
            from ..utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Successfully extracted content from {url}")
            return content_page

        except Exception as e:
            from ..utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Failed to extract content from {url}: {str(e)}")
            # Return a failed ContentPage
            return ContentPage(
                url=url,
                title="",
                content="",
                state=ContentPageState.FAILED  # Using the imported enum
            )

    def _extract_title(self, soup: BeautifulSoup) -> str:
        """Extract title from the page."""
        title_tag = soup.find('title')
        if title_tag:
            return title_tag.get_text().strip()

        # Fallback to h1 if title tag is not found
        h1_tag = soup.find('h1')
        if h1_tag:
            return h1_tag.get_text().strip()

        return "No Title"

    def _extract_content_and_headings(self, soup: BeautifulSoup) -> tuple[str, List[Heading]]:
        """Extract clean content and headings, filtering out navigation elements."""
        # Remove navigation, footer, and other UI elements that are not content
        for element in soup.find_all(['nav', 'header', 'footer', 'aside']):
            element.decompose()

        # Remove script and style elements
        for element in soup.find_all(['script', 'style']):
            element.decompose()

        # Remove elements with common CSS classes that indicate navigation/UI
        for element in soup.find_all(class_=lambda x: x and any(
            nav_class in x.lower() for nav_class in [
                'nav', 'navigation', 'menu', 'sidebar', 'footer',
                'header', 'topbar', 'breadcrumb', 'search'
            ]
        )):
            element.decompose()

        # Find content areas specific to Docusaurus
        content_selectors = [
            '.markdown',  # Docusaurus markdown content
            'article',    # General article content
            '[role="main"]',  # Main content area
            '.container',  # Container that might hold content
            '.main-content',  # Common main content class
            '.content',  # General content class
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        if not content_element:
            # If no specific content area found, use body
            content_element = soup.find('body')

        if content_element:
            # Extract headings
            headings = []
            for i, heading in enumerate(content_element.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])):
                level = int(heading.name[1])  # Extract number from h1, h2, etc.
                text = heading.get_text().strip()
                position = len(headings)  # Simplified position tracking
                heading_obj = Heading(level=level, text=text, position=position)
                headings.append(heading_obj)

            # Get text content, preserving structure
            content = content_element.get_text(separator='\n', strip=True)
            return content, headings
        else:
            # If no content area found, extract all text
            content = soup.get_text(separator='\n', strip=True)
            return content, []

    def extract_from_sitemap(self, sitemap_url: str) -> List[ContentPage]:
        """
        Extract URLs from a sitemap and process them.

        Args:
            sitemap_url: URL of the sitemap.xml

        Returns:
            List of ContentPage objects
        """
        try:
            response = self.session.get(sitemap_url, timeout=self.config.request_timeout)
            response.raise_for_status()

            soup = BeautifulSoup(response.content, 'xml')  # Use XML parser for sitemap
            urls = []

            # Find all <loc> elements which contain URLs
            for loc in soup.find_all('loc'):
                url = loc.text.strip()
                if url.startswith(self.config.book_base_url):
                    urls.append(url)

            from ..utils.helpers import setup_logging
            logger = setup_logging()
            logger.info(f"Found {len(urls)} URLs in sitemap")

            # Process each URL
            content_pages = []
            for url in urls:
                content_page = self.extract_content_from_url(url)
                content_pages.append(content_page)

            return content_pages

        except Exception as e:
            from ..utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Failed to process sitemap {sitemap_url}: {str(e)}")
            return []