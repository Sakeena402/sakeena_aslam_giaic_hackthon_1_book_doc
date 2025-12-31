# Data Model: Modular Retrieval System

## Domain Entities

### ContentPage
Represents a single page of content extracted from a Docusaurus site.
- **url**: str - The source URL of the content
- **title**: str - The title of the page
- **content**: str - The extracted content text
- **headings**: List[Heading] - Headings found in the content
- **chapter**: Optional[str] - Chapter identifier
- **section**: Optional[str] - Section identifier
- **extracted_at**: datetime - Timestamp of extraction
- **source_format**: str - Format of source (default: "docusaurus")
- **state**: ContentPageState - Current state of the page

### ContentChunk
Represents a segment of content prepared for embedding.
- **id**: str - Unique identifier for the chunk
- **content**: str - The chunk content text
- **source_page_url**: str - URL of the source page
- **chunk_index**: int - Position index of the chunk
- **overlap_with_next**: Optional[str] - Overlapping content with next chunk
- **original_start_pos**: int - Starting position in original content
- **original_end_pos**: int - Ending position in original content
- **heading_context**: Optional[str] - Relevant heading context
- **state**: ContentChunkState - Current state of the chunk

### EmbeddingVector
Represents the vector embedding of a content chunk.
- **chunk_id**: str - ID of the source chunk
- **vector**: List[float] - The embedding vector values
- **model_name**: str - Name of the embedding model used
- **model_version**: str - Version of the embedding model
- **embedding_created_at**: datetime - Timestamp of embedding creation
- **state**: EmbeddingVectorState - Current state of the embedding

### StoredVector
Represents a vector stored in Qdrant with metadata.
- **id**: str - Unique identifier in the vector database
- **vector**: List[float] - The embedding vector values
- **url**: str - Source URL of the content
- **title**: str - Title of the content
- **chunk_index**: int - Index of the chunk
- **content_preview**: str - Preview of the content
- **source_created_at**: datetime - Timestamp when source was created
- **chapter**: Optional[str] - Chapter identifier
- **section**: Optional[str] - Section identifier
- **collection_name**: str - Name of the Qdrant collection
- **state**: StoredVectorState - Current state of the stored vector

### Query
Represents a search query input to the retrieval system.
- **text**: str - The query text
- **created_at**: datetime - Timestamp of query creation
- **embedding**: Optional[List[float]] - Generated embedding vector
- **search_params**: Optional[Dict[str, Any]] - Additional search parameters

### RetrievedChunk
Represents a content chunk retrieved from the vector database.
- **id**: str - Unique identifier of the retrieved chunk
- **content**: str - The retrieved content
- **similarity_score**: float - Similarity score (0-1)
- **metadata**: Dict[str, Any] - Metadata associated with the chunk
- **retrieved_at**: datetime - Timestamp of retrieval

### ValidationResult
Represents the validation results for retrieved content.
- **query_text**: str - Original query text
- **retrieved_chunks**: List[RetrievedChunk] - Chunks that were validated
- **semantic_relevance_score**: float - Relevance score (0-1)
- **metadata_accuracy**: float - Metadata accuracy score (0-1)
- **latency_ms**: float - Validation latency in milliseconds
- **consistency_score**: float - Consistency score (0-1)
- **validation_timestamp**: datetime - Timestamp of validation
- **notes**: Optional[str] - Additional validation notes

### RetrievalRequest
Represents a complete retrieval request with parameters.
- **query**: Query - The query object
- **request_id**: str - Unique request identifier
- **top_k**: int - Number of results to return
- **similarity_threshold**: Optional[float] - Minimum similarity threshold
- **include_metadata**: bool - Whether to include metadata in results

### RetrievalResponse
Represents the complete response from a retrieval request.
- **request_id**: str - Unique request identifier
- **retrieved_chunks**: List[RetrievedChunk] - Retrieved content chunks
- **query_time_ms**: float - Query execution time in milliseconds
- **status**: str - Status ("success", "partial", "error")
- **total_results**: int - Total number of results
- **error_message**: Optional[str] - Error message if status is "error"

## State Enums

### ContentPageState
- PENDING_EXTRACTION
- EXTRACTING
- EXTRACTED
- FAILED

### ContentChunkState
- PENDING_CHUNKING
- CHUNKING
- CHUNKED
- FAILED

### EmbeddingVectorState
- PENDING_EMBEDDING
- EMBEDDING
- EMBEDDED
- FAILED

### StoredVectorState
- PENDING_STORAGE
- STORING
- STORED
- FAILED

## Configuration Model

### Config
Configuration class to manage settings and environment variables.
- **cohere_api_key**: str - Cohere API key
- **qdrant_url**: str - Qdrant database URL
- **qdrant_api_key**: str - Qdrant API key
- **book_base_url**: str - Base URL for the book
- **qdrant_collection_name**: str - Name of the Qdrant collection
- **chunk_size**: int - Size of content chunks (default: 1000)
- **overlap_size**: int - Size of overlap between chunks (default: 200)
- **request_timeout**: int - Request timeout in seconds (default: 30)
- **max_retries**: int - Maximum number of retries (default: 3)
- **retry_delay**: float - Delay between retries (default: 1.0)
- **retrieval_top_k**: int - Default number of results to return (default: 5)
- **retrieval_similarity_threshold**: float - Default similarity threshold (default: 0.5)
- **retrieval_model_name**: str - Default retrieval model name (default: "embed-multilingual-v3.0")