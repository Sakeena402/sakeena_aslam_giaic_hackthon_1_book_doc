# Quickstart: Modular Retrieval System

## Overview
This guide provides a quick setup and usage guide for the modular retrieval, chunking, and embedding validation system.

## Prerequisites
- Python 3.11+
- pip package manager
- Git

## Environment Setup

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Create virtual environment**:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**:
   Create a `.env` file in the root directory with the following:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_url
   QDRANT_API_KEY=your_qdrant_api_key
   BOOK_BASE_URL=https://your-book-url.com
   QDRANT_COLLECTION_NAME=book_content
   ```

## Usage Examples

### 1. Content Ingestion and Embedding
Process content from URLs and store embeddings in Qdrant:

```bash
cd backend
python main.py --urls https://example.com/page1 https://example.com/page2 --chunk-size 1000 --overlap-size 200
```

### 2. Content Ingestion from Sitemap
Process all content from a sitemap:

```bash
python main.py --sitemap https://example.com/sitemap.xml
```

### 3. Query Retrieval
Perform a retrieval query against the stored vectors:

```bash
python main.py --query "What are the core principles of RAG systems?" --top-k 5
```

### 4. Batch Query Testing
Run multiple retrieval queries:

```bash
python main.py --queries "What is semantic search?" "Explain embeddings" --top-k 3
```

### 5. Comprehensive Testing
Run comprehensive retrieval validation tests:

```bash
python main.py --test-retrieval --num-tests 10
```

### 6. Health Check
Check the health of the Qdrant connection:

```bash
python main.py --health
```

## Module Structure

The system is organized into these main modules:

- `services/ingestion.py` - Content extraction and URL processing
- `services/chunking.py` - Content segmentation and overlap logic
- `services/embedding.py` - Embedding generation and validation
- `services/storage.py` - Qdrant operations and vector storage
- `services/retrieval.py` - Query processing and similarity search
- `services/validation.py` - Validation logic for embeddings and chunks

## Configuration

The system uses a `Config` class that loads settings from environment variables. Key configuration options:

- `chunk_size`: Size of content chunks (default: 1000)
- `overlap_size`: Overlap between chunks (default: 200)
- `top_k`: Number of results to return (default: 5)
- `similarity_threshold`: Minimum similarity for results (default: 0.5)

## Validation

The system includes comprehensive validation for:

- Semantic relevance of retrieved chunks
- Metadata accuracy
- Consistency of results across multiple queries
- Embedding dimension validation
- Chunk integrity validation

Run validation with:
```bash
python main.py --query "test query" --validate-relevance
```

## Testing

Run the test suite:

```bash
# Run all tests
pytest

# Run specific module tests
pytest tests/unit/test_retrieval.py

# Run integration tests
pytest tests/integration/
```