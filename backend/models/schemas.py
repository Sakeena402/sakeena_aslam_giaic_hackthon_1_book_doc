"""
API schemas for the Modular Retrieval System.

This module contains request/response schemas for API endpoints.
"""

from dataclasses import dataclass, field
from typing import List, Optional
from datetime import datetime
from .dataclasses import RetrievedChunk


@dataclass
class HealthCheckResponse:
    """Response schema for health check endpoint."""
    service: str
    status: str
    database_connection: str
    qdrant_collection: str
    total_vectors: int
    last_heartbeat: str
    response_time_ms: float
    error: Optional[str] = None


@dataclass
class QueryRequest:
    """Request schema for query endpoint."""
    query: str
    top_k: int = 5
    similarity_threshold: float = 0.5


@dataclass
class QueryResponse:
    """Response schema for query endpoint."""
    request_id: str
    results: List[RetrievedChunk]
    query_time_ms: float
    status: str
    total_results: int
    error_message: Optional[str] = None


@dataclass
class ValidationRequest:
    """Request schema for validation endpoint."""
    query: str
    expected_topics: Optional[List[str]] = None


@dataclass
class ValidationResponse:
    """Response schema for validation endpoint."""
    query_text: str
    semantic_relevance_score: float
    metadata_accuracy: float
    latency_ms: float
    consistency_score: float
    validation_timestamp: datetime = field(default_factory=datetime.now)
    notes: Optional[str] = None