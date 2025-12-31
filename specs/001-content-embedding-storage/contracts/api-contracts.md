# API Contracts: Content Extraction, Embedding Generation, and Vector Storage

## Overview
This document defines the API contracts for the content extraction, embedding generation, and vector storage system. These contracts define the interfaces between different components of the system.

## 1. Content Extraction Service

### Extract Content from URLs
```
POST /extract
```

**Description**: Extract clean content from a list of URLs

**Request**:
```json
{
  "urls": ["https://example.com/page1", "https://example.com/page2"],
  "options": {
    "include_headings": true,
    "remove_navigation": true,
    "timeout": 30
  }
}
```

**Response**:
```json
{
  "success": true,
  "extracted_pages": [
    {
      "url": "https://example.com/page1",
      "title": "Page Title",
      "content": "Extracted clean text content...",
      "headings": [
        {
          "level": 1,
          "text": "Main Heading",
          "position": 0
        }
      ],
      "metadata": {
        "chapter": "Chapter Name",
        "section": "Section Name",
        "extracted_at": "2025-12-31T10:00:00Z",
        "source_format": "docusaurus"
      }
    }
  ],
  "errors": [],
  "stats": {
    "total_urls": 2,
    "successful": 2,
    "failed": 0
  }
}
```

**Error Response**:
```json
{
  "success": false,
  "error": "Error message",
  "details": {}
}
```

## 2. Content Chunking Service

### Chunk Content
```
POST /chunk
```

**Description**: Split content into appropriately sized chunks with overlap

**Request**:
```json
{
  "content_pages": [
    {
      "url": "https://example.com/page1",
      "title": "Page Title",
      "content": "Content to be chunked...",
      "metadata": {}
    }
  ],
  "options": {
    "chunk_size": 1000,
    "overlap_size": 200,
    "respect_boundaries": true
  }
}
```

**Response**:
```json
{
  "success": true,
  "chunks": [
    {
      "id": "chunk-1",
      "content": "Chunked content text...",
      "source_page_url": "https://example.com/page1",
      "chunk_index": 0,
      "overlap_with_next": "Overlapping text...",
      "metadata": {
        "original_start_pos": 0,
        "original_end_pos": 1000,
        "heading_context": "Main Heading"
      }
    }
  ],
  "stats": {
    "total_pages": 1,
    "total_chunks": 5,
    "avg_chunk_size": 1000
  }
}
```

## 3. Embedding Generation Service

### Generate Embeddings
```
POST /embed
```

**Description**: Generate vector embeddings for content chunks

**Request**:
```json
{
  "chunks": [
    {
      "id": "chunk-1",
      "content": "Content to embed...",
      "source_page_url": "https://example.com/page1"
    }
  ],
  "options": {
    "model": "embed-multilingual-v3.0",
    "input_type": "search_document"
  }
}
```

**Response**:
```json
{
  "success": true,
  "embeddings": [
    {
      "chunk_id": "chunk-1",
      "vector": [0.1, 0.2, 0.3, ...], // 1024-dimensional vector
      "model_name": "embed-multilingual-v3.0",
      "model_version": "3.0",
      "embedding_created_at": "2025-12-31T10:00:00Z"
    }
  ],
  "stats": {
    "total_chunks": 5,
    "successful": 5,
    "failed": 0
  }
}
```

## 4. Vector Storage Service

### Store Vectors
```
POST /store
```

**Description**: Store embeddings in Qdrant vector database with metadata

**Request**:
```json
{
  "embeddings": [
    {
      "chunk_id": "chunk-1",
      "vector": [0.1, 0.2, 0.3, ...],
      "model_name": "embed-multilingual-v3.0"
    }
  ],
  "options": {
    "collection_name": "book_content",
    "batch_size": 100
  }
}
```

**Response**:
```json
{
  "success": true,
  "stored_vectors": [
    {
      "id": "qdrant-point-id-1",
      "vector": [0.1, 0.2, 0.3, ...],
      "payload": {
        "url": "https://example.com/page1",
        "chapter": "Chapter Name",
        "section": "Section Name",
        "title": "Page Title",
        "chunk_index": 0,
        "content_preview": "First 100 chars of content...",
        "source_created_at": "2025-12-31T10:00:00Z"
      },
      "collection_name": "book_content"
    }
  ],
  "stats": {
    "total_vectors": 5,
    "successful": 5,
    "failed": 0
  }
}
```

## 5. Search Service

### Similarity Search
```
POST /search
```

**Description**: Perform similarity search against stored vectors

**Request**:
```json
{
  "query": "Search query text",
  "options": {
    "limit": 5,
    "with_payload": true,
    "with_vectors": false
  }
}
```

**Response**:
```json
{
  "success": true,
  "results": [
    {
      "id": "qdrant-point-id-1",
      "score": 0.95,
      "payload": {
        "url": "https://example.com/page1",
        "chapter": "Chapter Name",
        "section": "Section Name",
        "title": "Page Title",
        "content_preview": "Relevant content preview..."
      }
    }
  ],
  "query_embedding": [0.4, 0.5, 0.6, ...]
}
```

## 6. Validation Service

### Validate Pipeline
```
GET /validate
```

**Description**: Validate the complete pipeline by running a sample search

**Response**:
```json
{
  "success": true,
  "validation_results": {
    "content_extraction": {
      "status": "completed",
      "pages_extracted": 10
    },
    "chunking": {
      "status": "completed",
      "chunks_created": 50
    },
    "embedding": {
      "status": "completed",
      "vectors_generated": 50
    },
    "storage": {
      "status": "completed",
      "vectors_stored": 50
    },
    "search": {
      "status": "functional",
      "sample_query": "What is the main concept?",
      "sample_result": {
        "relevance_score": 0.92,
        "source_url": "https://example.com/page1"
      }
    }
  },
  "overall_status": "ready"
}
```