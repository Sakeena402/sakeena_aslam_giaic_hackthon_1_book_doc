# Quickstart Guide: Content Extraction, Embedding Generation, and Vector Storage

## Overview
This guide provides instructions for setting up and running the content extraction, embedding generation, and vector storage system. The system extracts content from Docusaurus sites, generates embeddings using Cohere, and stores them in Qdrant.

## Prerequisites

### System Requirements
- Python 3.11 or higher
- UV package manager installed
- Access to Cohere API (API key required)
- Access to Qdrant Cloud (URL and API key required)

### Environment Setup
1. Ensure Python 3.11+ is installed
2. Install UV package manager: `pip install uv`
3. Clone or access the project repository

## Setup Instructions

### 1. Project Structure
Create the project structure as follows:
```
backend/
├── main.py
├── pyproject.toml
├── .env
└── requirements.txt
```

### 2. Environment Variables
Create a `.env` file in the backend directory with the following variables:
```
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
QDRANT_API_KEY=your_qdrant_api_key_here
BOOK_BASE_URL=https://your-docusaurus-site.com
```

### 3. Dependencies
Create a `pyproject.toml` file with the required dependencies:
```toml
[project]
name = "content-embedding-storage"
version = "0.1.0"
dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=5.0.0",
    "qdrant-client>=1.8.0",
    "python-dotenv>=1.0.0",
    "pytest>=7.4.0"
]
```

Install dependencies using UV:
```bash
cd backend
uv sync
```

## Running the System

### 1. Basic Execution
To run the complete pipeline from URL extraction to vector storage:

```bash
cd backend
uv run python main.py
```

### 2. Command Line Options
The system supports various command-line options:

```bash
# Extract from specific URLs
uv run python main.py --urls "https://example.com/page1" "https://example.com/page2"

# Extract from sitemap
uv run python main.py --sitemap "https://example.com/sitemap.xml"

# Process with custom chunk size
uv run python main.py --chunk-size 1500 --overlap-size 300

# Validate with similarity search
uv run python main.py --validate
```

### 3. Expected Output
When running successfully, the system will:
1. Extract content from all specified URLs
2. Process and chunk the content
3. Generate embeddings using Cohere
4. Store vectors in Qdrant with metadata
5. Display progress and statistics
6. Optionally run validation with sample searches

## Configuration

### Main.py Configuration
The main.py file contains several configurable parameters:

```python
# Content extraction settings
CHUNK_SIZE = 1000  # Characters per chunk
OVERLAP_SIZE = 200  # Overlap between chunks
EXTRACT_TIMEOUT = 30  # Seconds to wait for each page
MAX_RETRIES = 3  # Number of retry attempts

# Cohere settings
COHERE_MODEL = "embed-multilingual-v3.0"
COHERE_INPUT_TYPE = "search_document"

# Qdrant settings
QDRANT_COLLECTION_NAME = "book_content"
VECTOR_DIMENSION = 1024  # For Cohere multilingual model
```

### Environment Variables
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant cloud instance URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `BOOK_BASE_URL`: Base URL of the Docusaurus site to extract from
- `QDRANT_COLLECTION_NAME`: Name of the Qdrant collection (optional, defaults to "book_content")

## Validation and Testing

### 1. Running Tests
Execute the test suite to verify functionality:

```bash
uv run pytest tests/ -v
```

### 2. Manual Validation
After running the system, you can validate the results by:

1. Checking Qdrant dashboard to confirm vectors are stored
2. Running a sample similarity search
3. Verifying metadata is correctly stored

### 3. Sample Validation Command
Run validation only:

```bash
uv run python main.py --validate --query "your sample query here"
```

## Troubleshooting

### Common Issues

#### Network Errors
- Ensure URLs are accessible
- Check network connectivity
- Verify appropriate request headers are used

#### API Limitations
- Monitor Cohere and Qdrant rate limits
- Implement appropriate delays if needed
- Check API key validity

#### Content Extraction Issues
- Verify CSS selectors match the target Docusaurus site
- Check for JavaScript-rendered content
- Review extracted content quality

### Error Handling
The system includes comprehensive error handling:
- Network timeouts are retried with exponential backoff
- Individual page failures don't stop the entire process
- Detailed logs are provided for debugging
- Progress is tracked and reported

## Next Steps

1. After successful setup, integrate with your RAG system
2. Monitor the Qdrant collection for performance
3. Adjust chunk sizes based on search quality
4. Set up scheduled runs if content updates frequently