# Data Model: Content Extraction, Embedding Generation, and Vector Storage

## Overview
This document defines the data models for the content extraction, embedding generation, and vector storage system. It covers the entities involved in the pipeline from URL ingestion to vector storage.

## Core Entities

### 1. ContentPage
Represents a single page of content extracted from a Docusaurus site

**Fields**:
- `url`: string (required) - The source URL of the content
- `title`: string (required) - The title of the page
- `content`: string (required) - The extracted clean text content
- `headings`: list of objects (optional) - Hierarchical headings structure
  - `level`: integer (required) - Heading level (1-6)
  - `text`: string (required) - Heading text
  - `position`: integer (required) - Position in content
- `metadata`: object (required) - Additional page metadata
  - `chapter`: string (optional) - Chapter name if applicable
  - `section`: string (optional) - Section name if applicable
  - `extracted_at`: datetime (required) - Timestamp of extraction
  - `source_format`: string (required) - Format of source (e.g., "docusaurus")

**Validation Rules**:
- URL must be a valid HTTP/HTTPS URL
- Content must not be empty
- Headings must have valid levels (1-6)
- Extracted_at must be a valid timestamp

### 2. ContentChunk
Represents a segment of content prepared for embedding

**Fields**:
- `id`: string (required) - Unique identifier for the chunk
- `content`: string (required) - The text content of the chunk
- `source_page_url`: string (required) - Reference to the source page
- `chunk_index`: integer (required) - Position of chunk within the source
- `overlap_with_next`: string (optional) - Overlapping text with next chunk
- `metadata`: object (required) - Chunk-specific metadata
  - `original_start_pos`: integer (required) - Start position in original content
  - `original_end_pos`: integer (required) - End position in original content
  - `heading_context`: string (optional) - Heading that provides context

**Validation Rules**:
- Content must not exceed embedding model limits (typically < 4096 tokens)
- Chunk index must be non-negative
- Source page URL must reference a valid ContentPage

### 3. EmbeddingVector
Represents the vector embedding of a content chunk

**Fields**:
- `chunk_id`: string (required) - Reference to the source chunk
- `vector`: list of floats (required) - The embedding vector values
- `model_name`: string (required) - Name of the embedding model used
- `model_version`: string (required) - Version of the embedding model
- `embedding_created_at`: datetime (required) - Timestamp of embedding creation

**Validation Rules**:
- Vector must have consistent dimensions (e.g., 1024 for Cohere embed-multilingual-v3.0)
- Model name and version must be valid Cohere model identifiers
- Embedding timestamp must be a valid datetime

### 4. StoredVector
Represents a vector stored in Qdrant with metadata

**Fields**:
- `id`: string (required) - Qdrant point ID
- `vector`: list of floats (required) - The embedding vector
- `payload`: object (required) - Metadata stored with the vector
  - `url`: string (required) - Source URL
  - `chapter`: string (optional) - Chapter name
  - `section`: string (optional) - Section name
  - `title`: string (required) - Page title
  - `chunk_index`: integer (required) - Position within source
  - `content_preview`: string (required) - Short preview of content
  - `source_created_at`: datetime (required) - When content was extracted
- `collection_name`: string (required) - Name of the Qdrant collection

**Validation Rules**:
- Payload must contain all required metadata fields
- Vector dimensions must match collection schema
- Collection name must be valid

## State Transitions

### ContentPage States
1. `pending_extraction` - URL identified, extraction not started
2. `extracting` - Currently being processed
3. `extracted` - Content successfully extracted
4. `failed` - Extraction failed

### ContentChunk States
1. `pending_chunking` - Content page ready for chunking
2. `chunking` - Currently being chunked
3. `chunked` - Successfully chunked
4. `failed` - Chunking failed

### EmbeddingVector States
1. `pending_embedding` - Chunk ready for embedding
2. `embedding` - Currently being embedded
3. `embedded` - Successfully embedded
4. `failed` - Embedding failed

### StoredVector States
1. `pending_storage` - Embedding ready for storage
2. `storing` - Currently being stored
3. `stored` - Successfully stored in Qdrant
4. `failed` - Storage failed

## Relationships

```
ContentPage (1) → (N) ContentChunk
ContentChunk (1) → (1) EmbeddingVector
EmbeddingVector (1) → (1) StoredVector
```

## API Contracts

### Content Extraction API
- Input: List of URLs
- Output: List of ContentPage objects
- Error handling: Return partial results with error details

### Content Chunking API
- Input: ContentPage object, chunk_size, overlap_size
- Output: List of ContentChunk objects
- Error handling: Return empty list on failure

### Embedding Generation API
- Input: List of ContentChunk objects
- Output: List of EmbeddingVector objects
- Error handling: Return partial results with error details

### Vector Storage API
- Input: List of EmbeddingVector objects
- Output: List of StoredVector objects
- Error handling: Return partial results with error details

### Similarity Search API
- Input: Query string, number of results
- Output: List of StoredVector objects with similarity scores
- Error handling: Return empty list on failure

## Validation Rules Summary

### Content Extraction
- All URLs must be accessible
- Extracted content must not be empty
- HTML structure must be preserved in metadata

### Content Chunking
- Chunks must not exceed embedding model limits
- Overlap must preserve context across boundaries
- Chunk boundaries should respect sentence boundaries where possible

### Embedding Generation
- All chunks must be successfully embedded or errors must be logged
- Embedding model parameters must be consistent
- Vector dimensions must match expected values

### Vector Storage
- All metadata must be stored with vectors
- Vector IDs must be unique within collection
- Storage must be confirmed before proceeding

### Search Validation
- Search must return relevant results
- Results must include proper metadata for traceability
- Search performance must be within acceptable limits