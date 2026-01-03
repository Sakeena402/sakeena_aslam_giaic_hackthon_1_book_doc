"""
Agent Service - Orchestrates the RAG agent functionality using OpenAI Assistants API

This module connects the OpenAI Assistants API agent with the existing Qdrant retrieval pipeline
and manages the overall agent workflow.
"""

import logging
from typing import List, Optional, Dict, Any
from datetime import datetime

from ..agent import RAGAgent
from ..models.dataclasses import Query as BackendQuery, RetrievedChunk, AgentQuery, AgentResponse
from ..config.settings import Config
from .retrieval import EnhancedQdrantRetriever


class AgentService:
    """Service class to orchestrate agent functionality and connect with retrieval pipeline."""

    def __init__(self, config: Config):
        self.config = config

        # Initialize the RAG Agent
        self.agent = RAGAgent(config)

        # Initialize supporting services
        self.retriever = EnhancedQdrantRetriever(config)

        # Set up logging
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

    @classmethod
    def from_config(cls):
        """Initialize the service with configuration from settings."""
        config = Config()
        return cls(config)

    def process_agent_query(
        self,
        query_text: str,
        query_type: str = "global",
        section_filter: Optional[str] = None,
        top_k: int = 5,
        similarity_threshold: float = 0.5
    ) -> AgentResponse:
        """
        Process an agent query through the full RAG pipeline using OpenAI Assistants API.

        Args:
            query_text: Natural language query from user
            query_type: "global" for full book search, "section_specific" for limited scope
            section_filter: Section to limit search to (required for section_specific queries)
            top_k: Number of results to retrieve
            similarity_threshold: Minimum similarity for inclusion

        Returns:
            AgentResponse with grounded response and supporting chunks
        """
        self.logger.info(f"Processing agent query via Assistants API: {query_text}")

        # Use the RAGAgent to process the query
        response = self.agent.process_query(
            query_text=query_text,
            query_type=query_type,
            section_filter=section_filter,
            top_k=top_k,
            similarity_threshold=similarity_threshold
        )

        return response

    def process_assistant_query(
        self,
        query_text: str,
        query_type: str = "global",
        section_filter: Optional[str] = None,
        top_k: int = 5,
        similarity_threshold: float = 0.5
    ) -> AgentResponse:
        """
        Alias for process_agent_query to maintain compatibility with test functions.
        """
        return self.process_agent_query(
            query_text=query_text,
            query_type=query_type,
            section_filter=section_filter,
            top_k=top_k,
            similarity_threshold=similarity_threshold
        )


def test_agent_service():
    """Test function to verify agent service functionality."""
    print("Testing Agent Service with OpenAI Assistants API...")

    try:
        # Initialize service
        service = AgentService.from_config()

        # Test global query
        print("\n1. Testing global query:")
        response = service.process_assistant_query(
            query_text="What is ROS 2?",
            query_type="global"
        )

        print(f"Query: What is ROS 2?")
        print(f"Response length: {len(response.response_text)} characters")
        print(f"Confidence: {response.confidence_score:.2f}")
        print(f"Grounding chunks: {len(response.grounding_chunks)}")

        # Test section-specific query
        print("\n2. Testing section-specific query:")
        response2 = service.process_assistant_query(
            query_text="Explain NVIDIA Isaac perception systems",
            query_type="section_specific",
            section_filter="isaac"
        )

        print(f"Query: Explain NVIDIA Isaac perception systems")
        print(f"Section filter: isaac")
        print(f"Response length: {len(response2.response_text)} characters")
        print(f"Confidence: {response2.confidence_score:.2f}")
        print(f"Grounding chunks: {len(response2.grounding_chunks)}")

        print("\n✅ Agent service tests completed successfully!")
        return True

    except Exception as e:
        print(f"❌ Error in agent service test: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    test_agent_service()