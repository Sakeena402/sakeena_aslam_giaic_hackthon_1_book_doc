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
            # Return a failed ContentPage with minimal valid content to avoid validation errors
            return ContentPage(
                url=url,
                title="Failed to load",
                content="Failed to load content",
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
        # Extract headings from the original soup to capture all headings
        headings = []
        all_headings = soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6'])
        for i, heading in enumerate(all_headings):
            level = int(heading.name[1])  # Extract number from h1, h2, etc.
            text = heading.get_text().strip()
            # Get the position of the heading in the original content
            position = 0
            if soup.body and text in soup.body.text:
                position = soup.body.text.find(text)
            heading_obj = Heading(level=level, text=text, position=position)
            headings.append(heading_obj)

        # First, try to find the main content area specifically
        # Look for Docusaurus-specific markdown containers
        content_selectors = [
            '.theme-doc-markdown',
            '.markdown',
            '[class*="markdown"]',
            '[class*="theme-doc-markdown"]',
            '[data-testid="doc-content"]',
            '.doc-content',
            '.docs-content',
            '.main-wrapper',  # Additional Docusaurus selector
            '[class*="docItem"]',  # Docusaurus document item
            '[class*="docMain"]',  # Docusaurus main content area
        ]

        content_element = None
        for selector in content_selectors:
            content_element = soup.select_one(selector)
            if content_element:
                break

        # If Docusaurus-specific selectors don't work, try more general ones
        if not content_element:
            general_selectors = [
                '[role="main"]',
                'article',
                '.main-content',
                '.content',
                'main',
                '.container',
                '.doc-wrapper',  # Additional selector
                '.content-wrapper',  # Additional selector
            ]
            for selector in general_selectors:
                content_element = soup.select_one(selector)
                if content_element:
                    break

        # If still no content found, try to find content by looking for text-rich elements
        if not content_element:
            # Look for elements with significant text content while avoiding navigation elements
            potential_content_elements = soup.find_all(['div', 'section', 'article', 'main'])
            for element in potential_content_elements:
                # Check if this element is likely to be content and not navigation
                element_classes = ' '.join(element.get('class', []))
                element_id = element.get('id', '')

                # Skip if this looks like navigation/sidebar
                if any(skip_class in element_classes.lower() for skip_class in [
                    'nav', 'navigation', 'menu', 'sidebar', 'footer', 'header', 'topbar',
                    'breadcrumb', 'search', 'toc', 'table-of-contents', 'pagination'
                ]) or any(skip_id in element_id.lower() for skip_id in [
                    'nav', 'navigation', 'menu', 'sidebar', 'footer', 'header', 'topbar'
                ]):
                    continue

                text_content = element.get_text(strip=True)
                # If this element has substantial text content, consider it as content
                if len(text_content) > 150:  # Increased minimum to ensure meaningful content
                    content_element = element
                    break

        # If still no content found, use body but filter out navigation elements
        if not content_element:
            content_element = soup.find('body')

        # Extract content based on where we found it
        if content_element:
            # Create a copy to work with to avoid modifying the original soup
            content_soup = BeautifulSoup(str(content_element), 'html.parser')

            # Remove navigation, footer, and other UI elements that are not content
            for element in content_soup.find_all(['nav', 'header', 'footer', 'aside']):
                element.decompose()

            # Remove script and style elements
            for element in content_soup.find_all(['script', 'style', 'noscript']):
                element.decompose()

            # Remove elements with common CSS classes that indicate navigation/UI
            for element in content_soup.find_all(class_=lambda x: x and any(
                nav_class in x.lower() for nav_class in [
                    'nav', 'navigation', 'menu', 'sidebar', 'footer',
                    'header', 'topbar', 'breadcrumb', 'search', 'table-of-contents',
                    'pagination', 'toc', 'sidebar', 'navigation', 'footer',
                    'navbar', 'menubar', 'sidebar-nav', 'nav-item', 'nav-link',
                    'mobile-nav', 'mobile-menu', 'drawer', 'modal', 'popup'
                ]
            )):
                element.decompose()

            # Remove elements with IDs that indicate navigation/UI
            for element in content_soup.find_all(id=lambda x: x and any(
                nav_id in x.lower() for nav_id in [
                    'nav', 'navigation', 'menu', 'sidebar', 'footer',
                    'header', 'topbar', 'breadcrumb', 'search', 'toc', 'table-of-contents',
                    'navbar', 'menubar', 'sidebar-nav', 'mobile-nav', 'mobile-menu', 'drawer'
                ]
            )):
                element.decompose()

            # Remove elements with data attributes that indicate UI elements
            elements_to_remove = []
            for element in content_soup.find_all():
                attrs = element.attrs
                if attrs:
                    for attr_name, attr_value in attrs.items():
                        if attr_name.startswith('data-'):
                            attr_val_str = str(attr_value).lower()
                            if 'nav' in attr_val_str or 'menu' in attr_val_str or 'sidebar' in attr_val_str:
                                elements_to_remove.append(element)
                                break
            for element in elements_to_remove:
                element.decompose()

            # Get the full text content from this element
            content = content_soup.get_text(separator='\n', strip=True)
        else:
            # If no specific content area found, extract all text from the entire page
            # but still filter out common navigation elements
            soup_copy = BeautifulSoup(str(soup), 'html.parser')

            # Remove navigation elements from the full soup copy
            for element in soup_copy.find_all(['nav', 'header', 'footer', 'aside']):
                element.decompose()

            for element in soup_copy.find_all(class_=lambda x: x and any(
                nav_class in x.lower() for nav_class in [
                    'nav', 'navigation', 'menu', 'sidebar', 'footer',
                    'header', 'topbar', 'breadcrumb', 'search', 'table-of-contents',
                    'pagination', 'toc', 'sidebar', 'navigation', 'footer'
                ]
            )):
                element.decompose()

            content = soup_copy.get_text(separator='\n', strip=True)

        # Clean up the content - remove excessive whitespace and newlines
        import re
        content = re.sub(r'\n\s*\n\s*\n+', '\n\n', content)  # Replace 3+ newlines with double newline
        content = re.sub(r'^\s+', '', content)  # Remove leading whitespace
        content = re.sub(r'\s+$', '', content)  # Remove trailing whitespace

        # Remove potential title/duplicate content that appears at the beginning
        # If content starts with the title, we might need to extract better
        title_match = soup.find('title')
        if title_match:
            title_text = title_match.get_text().strip()
            if content.startswith(title_text):
                # Remove the title from the beginning of content
                content = content[len(title_text):].strip()

        # Also remove common patterns that might be duplicated at the start
        h1_match = soup.find('h1')
        if h1_match:
            h1_text = h1_match.get_text().strip()
            if content.startswith(h1_text):
                # Remove the H1 from the beginning of content
                content = content[len(h1_text):].strip()
                # Add H1 as the first heading if not already present
                if h1_text and not any(h.text == h1_text for h in headings):
                    headings.insert(0, Heading(level=1, text=h1_text, position=0))

        # Final content cleaning and validation
        content = re.sub(r'\n\s*\n', '\n\n', content)  # Normalize multiple newlines to double

        # Ensure content is meaningful
        if len(content) < 50:
            # If content is still too short, try to get the largest text block
            # but only from elements that are likely to be actual content
            content_elements = soup.find_all(['p', 'div', 'section', 'article', 'main'])
            content_blocks = []

            for element in content_elements:
                element_text = element.get_text().strip()
                # Only include blocks that are substantial and not likely navigation
                if len(element_text) > 50 and not any(skip_class in ' '.join(element.get('class', [])).lower()
                                                    for skip_class in ['nav', 'menu', 'sidebar', 'footer', 'header']):
                    content_blocks.append(element_text)

            if content_blocks:
                # Use the largest content block
                content = max(content_blocks, key=len)
            else:
                # Final fallback - use the original approach but with better filtering
                content = soup.get_text(separator='\n', strip=True)
                # Remove common navigation patterns
                content = re.sub(r'\n\s*\n', '\n\n', content)
                content = content.strip()

        return content, headings

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
                # Only add pages that were successfully extracted or failed in a way that's still valid
                content_pages.append(content_page)

            # Filter out only the pages that were successfully extracted for the return
            # But keep failed ones to track which URLs had issues
            return content_pages

        except Exception as e:
            from ..utils.helpers import setup_logging
            logger = setup_logging()
            logger.error(f"Failed to process sitemap {sitemap_url}: {str(e)}")
            return []