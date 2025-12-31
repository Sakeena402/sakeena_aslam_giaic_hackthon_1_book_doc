# Quickstart Guide: Retrieval Pipeline and Validation

## Overview
This guide provides instructions for setting up and running the retrieval validation system. The system extends the existing content extraction system to add retrieval functionality that connects to Qdrant, performs similarity search, and validates results.

## Prerequisites

### System Requirements
- Python 3.11 or higher
- Access to Cohere API (API key from Spec 01)
- Access to Qdrant Cloud (URL and API key from Spec 01)
- Existing vector embeddings stored in Qdrant from Spec 01

### Environment Setup
1. Ensure Python 3.11+ is installed
2. Make sure you have the backend directory from Spec 01
3. Verify existing .env file has required credentials

## Setup Instructions

### 1. Verify Existing Setup
Ensure the following from Spec 01 are in place:
```
backend/
├── main.py
├── pyproject.toml
├── .env
└── requirements.txt
```

### 2. Environment Variables
Verify your `.env` file contains the required variables from Spec 01:
```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
BOOK_BASE_URL=https://your-docusaurus-site.com
QDRANT_COLLECTION_NAME=book_content
```

### 3. Dependencies
The system reuses the same dependencies as Spec 01. Ensure they are installed:
```bash
cd backend
pip install -r requirements.txt
```

## Running the Retrieval System

### 1. Basic Retrieval
To run the retrieval functionality with a sample query:

```bash
cd backend
python main.py --query "What are the core principles of RAG systems?" --validate
```

### 2. Batch Retrieval
To run multiple queries for validation:

```bash
python main.py --queries "What are the core principles of RAG systems?" "How does content chunking work?" --validate
```

### 3. Comprehensive Testing
To run comprehensive validation tests:

```bash
python main.py --test-retrieval --num-tests 10
```

### 4. Performance Testing
To test retrieval performance and latency:

```bash
python main.py --benchmark --num-queries 5
```

## Configuration Options

### Retrieval Parameters
- `--top-k`: Number of results to return (default: 5)
- `--similarity-threshold`: Minimum similarity threshold (default: 0.5)
- `--collection-name`: Qdrant collection name (default: from .env)

### Validation Parameters
- `--validate`: Enable result validation
- `--relevance-threshold`: Minimum relevance score (default: 0.8)
- `--test-topic`: Specific topic for testing (e.g., "RAG", "chunking", "embedding")

## Expected Output

When running retrieval, you should see output like:
```
Query: "What are the core principles of RAG systems?"
Retrieved 5 chunks in 150.2ms
Relevance Score: 0.87
Results:
1. [0.89] URL: https://book.com/intro-to-rag - Content about core RAG principles...
2. [0.85] URL: https://book.com/rag-fundamentals - Content about RAG architecture...
...
Validation: PASSED (87% relevance, 100% metadata accuracy)
```

## Validation Testing

### Running Test Queries
The system includes comprehensive test queries across different book topics:

```bash
python main.py --test-topic "RAG fundamentals"
python main.py --test-topic "content processing"
python main.py --test-topic "embedding techniques"
```

### Validation Metrics
The system measures:
- **Semantic Relevance**: How topically related results are to the query (target: >80%)
- **Metadata Accuracy**: Correctness of source attribution (target: 100%)
- **Latency**: Time to retrieve results (target: <1 second)
- **Consistency**: Repeatability of results across queries (target: >95%)

## Troubleshooting

### Common Issues

#### Qdrant Connection Issues
- Verify QDRANT_URL and QDRANT_API_KEY in .env
- Check that the collection name matches Spec 01
- Ensure the Qdrant collection exists and has embeddings

#### Cohere API Issues
- Verify COHERE_API_KEY is valid
- Check API rate limits
- Ensure the same embedding model is used as Spec 01

#### Empty Results
- Check that embeddings exist in Qdrant
- Verify the query is not too specific or broad
- Adjust similarity threshold if needed

### Error Handling
The system includes comprehensive error handling:
- Graceful degradation when Qdrant is unavailable
- Fallback responses when no relevant results found
- Detailed logging for debugging

## Next Steps

1. After successful validation, integrate with the RAG agent system
2. Monitor retrieval performance metrics
3. Adjust similarity thresholds based on validation results
4. Set up automated testing for ongoing validation