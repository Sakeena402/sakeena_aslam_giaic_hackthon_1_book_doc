"""
Agent-based RAG system using OpenRouter Chat Completions (Agent-style).
"""
from dotenv import load_dotenv
load_dotenv()
import logging
from typing import List, Optional
from datetime import datetime

from openai import OpenAI

from backend.config.settings import Config
from backend.models.dataclasses import RetrievedChunk, AgentResponse
from backend.services.retrieval import EnhancedQdrantRetriever
from dotenv import load_dotenv
load_dotenv()
# -----------------------------
# OpenRouter Configuration
# -----------------------------

import os
ROUTER_API_KEY = os.getenv("OPENROUTER_API_KEY", "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")

MODEL_NAME = "mistralai/devstral-2512:free"


class RAGAgent:
    """
    Agent-based RAG implementation using Chat Completions (OpenRouter-compatible).
    """

    def __init__(self, config: Config):
        self.config = config
        self.retriever = EnhancedQdrantRetriever(config)

        self.openai_client = OpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1",
    default_headers={
        "HTTP-Referer": "http://localhost",
        "X-Title": "RAG Agent"
    }
)
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.INFO)

    @classmethod
    def from_config(cls):
        return cls(Config())
    
    def _confidence(self, chunks: List[RetrievedChunk]) -> float:
        """
        Confidence score based on retrieval coverage.
        """
        if not chunks:
            return 0.0
        # Normalize confidence by chunk count (max = top_k)
        max_chunks = self.config.RETRIEVAL_TOP_K if hasattr(self.config, "RETRIEVAL_TOP_K") else 5
        return round(len(chunks) / max_chunks, 2)


    def process_query(
        self,
        query_text: str,
        query_type: str = "global",
        section_filter: Optional[str] = None,
        top_k: int = 5,
        similarity_threshold: float = 0.5,
    ) -> AgentResponse:
        """
        Full RAG pipeline: retrieve → ground → reason → generate
        """

        self.logger.info(f"Processing query: {query_text}")

        # ---------------- Retrieval ----------------
        filters = None
        if query_type == "section_specific" and section_filter:
            filters = {"section": section_filter}

        chunks: List[RetrievedChunk] = self.retriever.retrieve_similar_chunks(
            query_text=query_text,
            top_k=top_k,
            similarity_threshold=similarity_threshold,
            filters=filters,
        )

        if not chunks:
            return AgentResponse(
                id=f"resp_{hash(query_text)}",
                query_id=f"query_{hash(query_text)}",
                response_text="I could not find relevant information in the provided content.",
                grounding_chunks=[],
                confidence_score=0.0,
                generated_at=datetime.utcnow(),
            )

        # ---------------- Grounded Context ----------------
        context = "\n\n".join(
            f"[Source: {c.metadata.get('url', 'unknown')}]\n{c.content}"
            for c in chunks
        )

        prompt = f"""
You are an educational AI agent.

Rules:
- Use ONLY the context below.
- Do NOT use outside knowledge.
- If the answer is not in the context, say so clearly.
- Be factual and concise.

Context:
{context}

Question:
{query_text}

Answer:
"""

        # ---------------- Agent Reasoning ----------------
        completion = self.openai_client.chat.completions.create(
            model=MODEL_NAME,
            messages=[
                {
                    "role": "system",
                    "content": "You are a retrieval-grounded AI agent."
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            temperature=0.2,
        )

        response_text = completion.choices[0].message.content

        return AgentResponse(
            id=f"resp_{hash(query_text)}",
            query_id=f"query_{hash(query_text)}",
            response_text=response_text,
            grounding_chunks=chunks,
            confidence_score=self._confidence(chunks),
            generated_at=datetime.utcnow(),
        )  