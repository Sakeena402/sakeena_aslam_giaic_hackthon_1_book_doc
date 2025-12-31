# Data Model: Retrieval Pipeline and Validation

## Overview
This document defines the data models for the retrieval validation system. It covers the entities involved in the retrieval pipeline from query input to validated results with metadata.

## Core Entities

### 1. Query
Represents a search query input to the retrieval system

**Fields**:
- `text`: string (required) - The query text to search for
- `embedding`: list of floats (optional) - The vector embedding of the query
- `created_at`: datetime (required) - Timestamp of query creation
- `search_params`: object (optional) - Parameters for the search (top_k, threshold, etc.)

**Validation Rules**:
- Text must not be empty
- Embedding must have consistent dimensions (1024 for Cohere embed-multilingual-v3.0)
- Created_at must be a valid timestamp

### 2. RetrievedChunk
Represents a content chunk retrieved from the vector database

**Fields**:
- `id`: string (required) - Qdrant point ID
- `content`: string (required) - The text content of the retrieved chunk
- `similarity_score`: float (required) - Similarity score between query and chunk
- `metadata`: object (required) - Metadata from the vector database
  - `url`: string (required) - Source URL of the content
  - `chapter`: string (optional) - Chapter name if applicable
  - `section`: string (optional) - Section name if applicable
  - `title`: string (required) - Page title
  - `chunk_index`: integer (required) - Position within source
  - `content_preview`: string (required) - Short preview of content
- `retrieved_at`: datetime (required) - Timestamp of retrieval

**Validation Rules**:
- ID must be a valid Qdrant point ID
- Content must not be empty
- Similarity score must be between 0 and 1
- All required metadata fields must be present
- Retrieved_at must be a valid timestamp

### 3. ValidationResult
Represents the validation results for retrieved content

**Fields**:
- `query_text`: string (required) - The original query text
- `retrieved_chunks`: list of RetrievedChunk (required) - Retrieved results
- `semantic_relevance_score`: float (required) - Relevance score of results (0-1)
- `metadata_accuracy`: float (required) - Accuracy of metadata mapping (0-1)
- `latency_ms`: float (required) - Time taken for retrieval in milliseconds
- `consistency_score`: float (required) - Consistency across repeated queries (0-1)
- `validation_timestamp`: datetime (required) - When validation was performed
- `notes`: string (optional) - Additional validation notes

**Validation Rules**:
- Query text must match the original query
- Retrieved chunks must be from the same query execution
- All scores must be between 0 and 1
- Latency must be a positive number
- Validation timestamp must be a valid datetime

### 4. RetrievalRequest
Represents a complete retrieval request with parameters

**Fields**:
- `query`: Query (required) - The search query
- `top_k`: integer (required) - Number of results to return (default 5)
- `similarity_threshold`: float (optional) - Minimum similarity threshold (0-1)
- `include_metadata`: boolean (required) - Whether to include metadata (default true)
- `request_id`: string (required) - Unique identifier for the request

**Validation Rules**:
- Top_k must be a positive integer
- Similarity threshold must be between 0 and 1 if provided
- Request ID must be unique for the session

### 5. RetrievalResponse
Represents the complete response from a retrieval request

**Fields**:
- `request_id`: string (required) - ID of the corresponding request
- `retrieved_chunks`: list of RetrievedChunk (required) - Retrieved content chunks
- `total_results`: integer (required) - Total number of results returned
- `query_time_ms`: float (required) - Time taken for the query in milliseconds
- `status`: string (required) - Status of the retrieval ("success", "partial", "error")
- `error_message`: string (optional) - Error message if status is "error"

**Validation Rules**:
- Request ID must match the corresponding request
- Total results must match the length of retrieved chunks
- Query time must be a positive number
- Status must be one of the allowed values

## State Transitions

### Query States
1. `pending_embedding` - Query received, embedding not yet generated
2. `embedding` - Currently generating query embedding
3. `embedded` - Query embedding generated successfully
4. `failed` - Embedding generation failed

### Retrieval States
1. `pending_search` - Query embedding ready, search not started
2. `searching` - Currently performing similarity search
3. `retrieved` - Search completed successfully
4. `failed` - Search failed

### Validation States
1. `pending_validation` - Results retrieved, validation not started
2. `validating` - Currently validating results
3. `validated` - Validation completed
4. `failed` - Validation failed

## Relationships

```
Query (1) → (1) RetrievalRequest
RetrievalRequest (1) → (1) RetrievalResponse
RetrievalResponse (1) → (N) RetrievedChunk
RetrievedChunk (N) → (1) ValidationResult
```

## API Contracts

### Similarity Search API
- Input: Query object with text and optional parameters
- Output: RetrievalResponse object with retrieved chunks and metadata
- Error handling: Return error status with descriptive message

### Validation API
- Input: Query text and expected topics/chapters
- Output: ValidationResult object with relevance scores
- Error handling: Return validation failure with details

### Batch Retrieval API
- Input: List of Query objects
- Output: List of RetrievalResponse objects
- Error handling: Return partial results with error details for failed queries

## Validation Rules Summary

### Query Processing
- All queries must generate valid embeddings
- Query embeddings must match dimensions of stored document embeddings
- Query text must be properly sanitized

### Retrieval Process
- Retrieved chunks must have valid similarity scores
- All required metadata fields must be present in results
- Retrieval time must be within acceptable limits

### Validation Process
- Semantic relevance must be measured against expected topics
- Metadata accuracy must be verified against original sources
- Consistency must be validated across repeated queries
- Performance metrics must be collected and logged

### Result Quality
- Retrieved content must be semantically related to the query
- All results must include complete and accurate metadata
- Results must be ordered by relevance score