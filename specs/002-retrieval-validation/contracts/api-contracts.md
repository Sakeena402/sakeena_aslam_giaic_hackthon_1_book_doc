# API Contracts: Retrieval Pipeline and Validation

## Overview
This document defines the API contracts for the retrieval validation system. These contracts define the interfaces for connecting to Qdrant, performing similarity searches, and validating retrieved content.

## 1. Similarity Search Service

### Search for Relevant Chunks
```
POST /search
```

**Description**: Perform similarity search against stored embeddings and return relevant content chunks

**Request**:
```json
{
  "query": "What are the core principles of RAG systems?",
  "top_k": 5,
  "similarity_threshold": 0.5,
  "include_metadata": true
}
```

**Response**:
```json
{
  "request_id": "req-12345",
  "status": "success",
  "retrieved_chunks": [
    {
      "id": "qdrant-point-id-1",
      "content": "Retrieved content chunk text...",
      "similarity_score": 0.85,
      "metadata": {
        "url": "https://book-url.com/chapter1",
        "chapter": "Introduction to RAG",
        "section": "Core Principles",
        "title": "RAG Fundamentals",
        "chunk_index": 0,
        "content_preview": "Preview of the content chunk..."
      },
      "retrieved_at": "2025-12-31T10:00:00Z"
    }
  ],
  "total_results": 5,
  "query_time_ms": 120.5,
  "query_embedding": [0.1, 0.2, 0.3, ...]
}
```

**Error Response**:
```json
{
  "request_id": "req-12345",
  "status": "error",
  "error_message": "Error description",
  "query_time_ms": 0
}
```

## 2. Validation Service

### Validate Retrieval Results
```
POST /validate
```

**Description**: Validate that retrieved results are semantically relevant and properly attributed

**Request**:
```json
{
  "query_text": "What are the core principles of RAG systems?",
  "retrieved_chunks": [
    {
      "id": "qdrant-point-id-1",
      "content": "Retrieved content chunk text...",
      "similarity_score": 0.85,
      "metadata": {
        "url": "https://book-url.com/chapter1",
        "chapter": "Introduction to RAG",
        "section": "Core Principles",
        "title": "RAG Fundamentals",
        "chunk_index": 0,
        "content_preview": "Preview of the content chunk..."
      }
    }
  ],
  "expected_topics": ["RAG principles", "retrieval systems", "augmentation"]
}
```

**Response**:
```json
{
  "validation_result": {
    "query_text": "What are the core principles of RAG systems?",
    "semantic_relevance_score": 0.87,
    "metadata_accuracy": 1.0,
    "latency_ms": 120.5,
    "consistency_score": 0.95,
    "validation_timestamp": "2025-12-31T10:00:00Z",
    "notes": "Results are highly relevant to query topics",
    "passed_validation": true
  }
}
```

## 3. Batch Retrieval Service

### Batch Search for Multiple Queries
```
POST /batch-search
```

**Description**: Perform multiple similarity searches in a batch

**Request**:
```json
{
  "queries": [
    {
      "text": "What are the core principles of RAG systems?",
      "top_k": 3,
      "similarity_threshold": 0.5
    },
    {
      "text": "How does content chunking work?",
      "top_k": 3,
      "similarity_threshold": 0.5
    }
  ]
}
```

**Response**:
```json
{
  "batch_id": "batch-123",
  "total_queries": 2,
  "successful_queries": 2,
  "results": [
    {
      "request_id": "req-1",
      "status": "success",
      "retrieved_chunks": [
        {
          "id": "qdrant-point-id-1",
          "content": "Retrieved content chunk text...",
          "similarity_score": 0.85,
          "metadata": {
            "url": "https://book-url.com/chapter1",
            "chapter": "Introduction to RAG",
            "section": "Core Principles",
            "title": "RAG Fundamentals",
            "chunk_index": 0,
            "content_preview": "Preview of the content chunk..."
          },
          "retrieved_at": "2025-12-31T10:00:00Z"
        }
      ],
      "total_results": 3,
      "query_time_ms": 120.5
    },
    {
      "request_id": "req-2",
      "status": "success",
      "retrieved_chunks": [
        {
          "id": "qdrant-point-id-2",
          "content": "Content chunking involves breaking down...",
          "similarity_score": 0.78,
          "metadata": {
            "url": "https://book-url.com/chapter2",
            "chapter": "Content Processing",
            "section": "Chunking Strategies",
            "title": "Processing Techniques",
            "chunk_index": 1,
            "content_preview": "Preview of chunking content..."
          },
          "retrieved_at": "2025-12-31T10:00:01Z"
        }
      ],
      "total_results": 3,
      "query_time_ms": 95.2
    }
  ],
  "batch_completion_time_ms": 215.7
}
```

## 4. Connection Health Service

### Check Qdrant Connection
```
GET /health
```

**Description**: Check the health and connectivity of the Qdrant database

**Response**:
```json
{
  "service": "retrieval-service",
  "status": "healthy",
  "database_connection": "connected",
  "qdrant_collection": "book_content",
  "total_vectors": 1500,
  "last_heartbeat": "2025-12-31T10:00:00Z",
  "response_time_ms": 10.2
}
```

## 5. Performance Metrics Service

### Get Retrieval Metrics
```
GET /metrics
```

**Description**: Retrieve performance metrics for the retrieval system

**Response**:
```json
{
  "metrics": {
    "total_queries": 150,
    "avg_response_time_ms": 110.5,
    "p95_response_time_ms": 180.2,
    "success_rate": 0.98,
    "avg_similarity_score": 0.75,
    "validation_success_rate": 0.92,
    "last_updated": "2025-12-31T10:00:00Z"
  }
}
```

## 6. Test Validation Service

### Run Comprehensive Tests
```
POST /test
```

**Description**: Run comprehensive tests with multiple sample queries from different book topics

**Request**:
```json
{
  "test_queries": [
    {
      "topic": "RAG fundamentals",
      "query": "What are the core principles of RAG systems?",
      "expected_chapters": ["Introduction to RAG", "Core Concepts"]
    },
    {
      "topic": "Content processing",
      "query": "How does content chunking work?",
      "expected_chapters": ["Content Processing", "Chunking Strategies"]
    }
  ],
  "validation_threshold": 0.8
}
```

**Response**:
```json
{
  "test_id": "test-456",
  "total_tests": 2,
  "passed_tests": 2,
  "success_rate": 1.0,
  "average_relevance": 0.87,
  "metadata_accuracy": 1.0,
  "test_results": [
    {
      "query": "What are the core principles of RAG systems?",
      "relevance_score": 0.89,
      "passed": true,
      "details": "Retrieved relevant content from expected chapters"
    },
    {
      "query": "How does content chunking work?",
      "relevance_score": 0.85,
      "passed": true,
      "details": "Retrieved relevant content from expected chapters"
    }
  ],
  "summary": "All tests passed validation criteria",
  "completed_at": "2025-12-31T10:00:00Z"
}
```