# Modular Retrieval System

This system extracts content from Docusaurus sites, chunks it for semantic retrieval, generates embeddings using Cohere, and stores vectors with metadata in Qdrant.

## Architecture

The system is organized into modular components:

- **config/**: Configuration management
- **models/**: Data classes and schemas
- **services/**: Business logic modules
  - `ingestion.py`: Content extraction and URL processing
  - `chunking.py`: Content segmentation and overlap logic
  - `embedding.py`: Embedding generation and validation
  - `storage.py`: Qdrant operations and vector storage
  - `retrieval.py`: Query processing and similarity search
  - `validation.py`: Validation logic for embeddings and chunks
- **utils/**: Helper functions and utilities
- **tests/**: Unit and integration tests

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create a `.env` file with the following environment variables:
   ```
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   BOOK_BASE_URL=your_book_base_url
   QDRANT_COLLECTION_NAME=book_content
   ```

## Usage

### Content Extraction and Ingestion

Process specific URLs:
```bash
python main.py --urls https://example.com/page1 https://example.com/page2
```

Process from sitemap:
```bash
python main.py --sitemap https://example.com/sitemap.xml
```

Customize chunking parameters:
```bash
python main.py --urls https://example.com/page --chunk-size 2000 --overlap-size 400
```

### Content Retrieval

Query for relevant content:
```bash
python main.py --query "What are the core principles of RAG systems?"
```

Batch query multiple questions:
```bash
python main.py --queries "What is semantic search?" "How does chunking work?"
```

Customize retrieval parameters:
```bash
python main.py --query "Your question" --top-k 10 --similarity-threshold 0.7
```

### Validation and Testing

Run comprehensive retrieval tests:
```bash
python main.py --test-retrieval --num-tests 5
```

Run performance benchmarking:
```bash
python main.py --benchmark
```

Validate with semantic relevance checking:
```bash
python main.py --query "Your question" --validate-relevance
```

Check system health:
```bash
python main.py --health
```

### Available Options

- `--urls`: List of URLs to process
- `--sitemap`: Sitemap URL to parse for URLs
- `--chunk-size`: Size of content chunks (default: 1000)
- `--overlap-size`: Size of overlap between chunks (default: 200)
- `--query`: Test query for retrieval validation
- `--queries`: Multiple test queries for batch retrieval
- `--top-k`: Number of results to return (default: 5)
- `--similarity-threshold`: Minimum similarity threshold (default: 0.5)
- `--test-retrieval`: Run comprehensive retrieval validation tests
- `--num-tests`: Number of tests to run (default: 10)
- `--benchmark`: Run performance benchmarking
- `--health`: Check health of Qdrant connection
- `--validate-relevance`: Run semantic relevance validation
- `--validate`: Run validation after processing
- `--expected-topics`: Expected topics for relevance validation

## API Functions

The system exposes the following key functions for programmatic use:

- `perform_retrieval(config, query_text, top_k=5, similarity_threshold=0.5)`: Perform a complete retrieval operation
- `simple_retrieve_chunks(query_text, config, top_k=5, similarity_threshold=0.5)`: Simple retrieval function that returns relevant chunks
- `validate_retrieval_results(query_text, retrieved_chunks, config)`: Validate retrieval results

## Testing

Run the test suite:
```bash
# Run all tests
pytest

# Run unit tests only
pytest tests/unit/

# Run integration tests only
pytest tests/integration/

# Run with coverage
pytest --cov=backend/
```

## Environment Variables

- `COHERE_API_KEY`: Cohere API key for embedding generation
- `QDRANT_URL`: Qdrant database URL
- `QDRANT_API_KEY`: Qdrant API key
- `BOOK_BASE_URL`: Base URL for the book/content being indexed
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (default: book_content)