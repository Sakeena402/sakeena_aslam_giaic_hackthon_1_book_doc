"""
Data classes for the Modular Retrieval System.

This module contains all domain entities and state enums as defined in the data model.
"""

from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Optional, Any
from enum import Enum
import uuid


# State Enums
class ContentPageState(Enum):
    PENDING_EXTRACTION = "pending_extraction"
    EXTRACTING = "extracting"
    EXTRACTED = "extracted"
    FAILED = "failed"


class ContentChunkState(Enum):
    PENDING_CHUNKING = "pending_chunking"
    CHUNKING = "chunking"
    CHUNKED = "chunked"
    FAILED = "failed"


class EmbeddingVectorState(Enum):
    PENDING_EMBEDDING = "pending_embedding"
    EMBEDDING = "embedding"
    EMBEDDED = "embedded"
    FAILED = "failed"


class StoredVectorState(Enum):
    PENDING_STORAGE = "pending_storage"
    STORING = "storing"
    STORED = "stored"
    FAILED = "failed"


# Domain Entities
@dataclass
class Heading:
    """Represents a heading in content."""
    level: int
    text: str
    position: int

    def __post_init__(self):
        if not 1 <= self.level <= 6:
            raise ValueError(f"Heading level must be between 1 and 6, got {self.level}")


@dataclass
class ContentPage:
    """Represents a single page of content extracted from a Docusaurus site."""
    url: str
    title: str
    content: str
    headings: List[Heading] = field(default_factory=list)
    chapter: Optional[str] = None
    section: Optional[str] = None
    extracted_at: datetime = field(default_factory=datetime.now)
    source_format: str = "docusaurus"
    state: ContentPageState = ContentPageState.PENDING_EXTRACTION

    def __post_init__(self):
        """Validate required fields and content."""
        if not self.url:
            raise ValueError("URL is required")
        # Only require content if the page was successfully extracted
        if not self.content and self.state != ContentPageState.FAILED:
            raise ValueError("Content cannot be empty")
        if not self.title and self.state != ContentPageState.FAILED:
            raise ValueError("Title is required")

        # Validate URL format
        from urllib.parse import urlparse
        parsed = urlparse(self.url)
        if not parsed.scheme or not parsed.netloc:
            raise ValueError(f"Invalid URL format: {self.url}")


@dataclass
class ContentChunk:
    """Represents a segment of content prepared for embedding."""
    id: str
    content: str
    source_page_url: str
    chunk_index: int
    overlap_with_next: Optional[str] = None
    original_start_pos: int = 0
    original_end_pos: int = 0
    heading_context: Optional[str] = None
    state: ContentChunkState = ContentChunkState.PENDING_CHUNKING

    def __post_init__(self):
        """Validate required fields and content."""
        if not self.id:
            raise ValueError("ID is required")
        if not self.content:
            raise ValueError("Content cannot be empty")
        if not self.source_page_url:
            raise ValueError("Source page URL is required")
        if self.chunk_index < 0:
            raise ValueError("Chunk index must be non-negative")


@dataclass
class EmbeddingVector:
    """Represents the vector embedding of a content chunk."""
    chunk_id: str
    vector: List[float]
    model_name: str
    model_version: str
    embedding_created_at: datetime = field(default_factory=datetime.now)
    state: EmbeddingVectorState = EmbeddingVectorState.PENDING_EMBEDDING

    def __post_init__(self):
        """Validate required fields and vector dimensions."""
        if not self.chunk_id:
            raise ValueError("Chunk ID is required")
        if not self.vector:
            raise ValueError("Vector cannot be empty")
        if not self.model_name:
            raise ValueError("Model name is required")
        if not self.model_version:
            raise ValueError("Model version is required")


@dataclass
class StoredVector:
    """Represents a vector stored in Qdrant with metadata."""
    id: str
    vector: List[float]
    url: str
    title: str
    chunk_index: int
    content_preview: str
    source_created_at: datetime
    page_title: Optional[str] = None  # New field for page title
    section_heading: Optional[str] = None  # New field for section heading
    chapter: Optional[str] = None
    section: Optional[str] = None
    collection_name: str = "book_content"
    state: StoredVectorState = StoredVectorState.PENDING_STORAGE

    def __post_init__(self):
        """Validate required fields."""
        if not self.id:
            raise ValueError("ID is required")
        if not self.vector:
            raise ValueError("Vector cannot be empty")
        if not self.url:
            raise ValueError("URL is required")
        if not self.title:
            raise ValueError("Title is required")
        if not self.content_preview:
            raise ValueError("Content preview is required")


@dataclass
class Query:
    """Represents a search query input to the retrieval system."""
    text: str
    created_at: datetime = field(default_factory=datetime.now)
    embedding: Optional[List[float]] = None
    search_params: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate required fields."""
        if not self.text:
            raise ValueError("Text is required")
        if self.embedding is not None and len(self.embedding) == 0:
            raise ValueError("Embedding cannot be empty")
        # Note: Embedding dimension validation (1024 for Cohere embed-multilingual-v3.0)
        # will be done when embedding is generated


@dataclass
class RetrievedChunk:
    """Represents a content chunk retrieved from the vector database."""
    id: str
    content: str
    similarity_score: float
    metadata: Dict[str, Any]
    retrieved_at: datetime = field(default_factory=datetime.now)

    def __post_init__(self):
        """Validate required fields."""
        if not self.id:
            raise ValueError("ID is required")
        if not self.content:
            raise ValueError("Content is required")
        if not 0 <= self.similarity_score <= 1:
            raise ValueError("Similarity score must be between 0 and 1")
        if not self.metadata:
            raise ValueError("Metadata is required")
        # Validate required metadata fields
        required_metadata = ["url", "title", "chunk_index", "content_preview"]
        for field_name in required_metadata:
            if field_name not in self.metadata:
                raise ValueError(f"Metadata field '{field_name}' is required")


@dataclass
class ValidationResult:
    """Represents the validation results for retrieved content."""
    query_text: str
    retrieved_chunks: List[RetrievedChunk]
    semantic_relevance_score: float
    metadata_accuracy: float
    latency_ms: float
    consistency_score: float
    validation_timestamp: datetime = field(default_factory=datetime.now)
    notes: Optional[str] = None

    def __post_init__(self):
        """Validate required fields."""
        if not self.query_text:
            raise ValueError("Query text is required")
        if not 0 <= self.semantic_relevance_score <= 1:
            raise ValueError("Semantic relevance score must be between 0 and 1")
        if not 0 <= self.metadata_accuracy <= 1:
            raise ValueError("Metadata accuracy must be between 0 and 1")
        if self.latency_ms < 0:
            raise ValueError("Latency must be a positive number")
        if not 0 <= self.consistency_score <= 1:
            raise ValueError("Consistency score must be between 0 and 1")


@dataclass
class RetrievalRequest:
    """Represents a complete retrieval request with parameters."""
    query: Query
    request_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    top_k: int = 5
    similarity_threshold: Optional[float] = 0.5
    include_metadata: bool = True

    def __post_init__(self):
        """Validate required fields."""
        if self.top_k <= 0:
            raise ValueError("Top_k must be a positive integer")
        if self.similarity_threshold is not None and not 0 <= self.similarity_threshold <= 1:
            raise ValueError("Similarity threshold must be between 0 and 1")


@dataclass
class RetrievalResponse:
    """Represents the complete response from a retrieval request."""
    request_id: str
    retrieved_chunks: List[RetrievedChunk]
    query_time_ms: float
    status: str
    total_results: int = 0
    error_message: Optional[str] = None

    def __post_init__(self):
        """Validate required fields."""
        if not self.request_id:
            raise ValueError("Request ID is required")
        if self.query_time_ms < 0:
            raise ValueError("Query time must be a positive number")
        if self.status not in ["success", "partial", "error"]:
            raise ValueError("Status must be one of: success, partial, error")
        if self.total_results != len(self.retrieved_chunks):
            raise ValueError("Total results must match the length of retrieved chunks")


@dataclass
class AgentQuery:
    """Represents a query submitted to the AI agent"""
    query_text: str
    query_type: str = "global"  # "global" or "section_specific"
    section_filter: Optional[str] = None
    created_at: datetime = field(default_factory=datetime.now)
    user_context: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        """Validate required fields."""
        if not self.query_text or len(self.query_text.strip()) < 3:
            raise ValueError("Query text must be at least 3 characters")
        if self.query_type not in ["global", "section_specific"]:
            raise ValueError("Query type must be either 'global' or 'section_specific'")


@dataclass
class AgentResponse:
    """Represents the final response generated by the AI agent"""
    id: str
    query_id: str
    response_text: str
    grounding_chunks: List[RetrievedChunk] = field(default_factory=list)
    confidence_score: float = 0.0
    generated_at: datetime = field(default_factory=datetime.now)
    tokens_used: int = 0

    def __post_init__(self):
        """Validate required fields."""
        if not self.id:
            raise ValueError("ID is required")
        if not self.query_id:
            raise ValueError("Query ID is required")
        if not self.response_text:
            raise ValueError("Response text is required")
        if not 0 <= self.confidence_score <= 1:
            raise ValueError("Confidence score must be between 0 and 1")