"""
Configuration management for the Modular Retrieval System.

This module contains the Config class to manage settings and environment variables.
"""

import os
from dataclasses import dataclass, field


@dataclass
class Config:
    """Configuration class to manage settings and environment variables."""

    cohere_api_key: str = field(default_factory=lambda: os.getenv("COHERE_API_KEY", ""))
    qdrant_url: str = field(default_factory=lambda: os.getenv("QDRANT_URL", ""))
    qdrant_api_key: str = field(default_factory=lambda: os.getenv("QDRANT_API_KEY", ""))
    book_base_url: str = field(default_factory=lambda: os.getenv("BOOK_BASE_URL", ""))
    qdrant_collection_name: str = field(default_factory=lambda: os.getenv("QDRANT_COLLECTION_NAME", "book_content"))

    # Processing settings
    chunk_size: int = 1000
    overlap_size: int = 200
    request_timeout: int = 30
    max_retries: int = 3
    retry_delay: float = 1.0

    # Retrieval settings
    retrieval_top_k: int = 5
    retrieval_similarity_threshold: float = 0.5
    retrieval_model_name: str = "embed-multilingual-v3.0"

    def __post_init__(self):
        """Validate required configuration values."""
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")
        if not self.book_base_url:
            raise ValueError("BOOK_BASE_URL environment variable is required")