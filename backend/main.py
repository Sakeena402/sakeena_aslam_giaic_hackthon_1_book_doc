#!/usr/bin/env python3
"""
Content Extraction, Embedding Generation, and Vector Storage System

This system extracts content from Docusaurus sites, chunks it for semantic retrieval,
generates embeddings using Cohere, and stores vectors with metadata in Qdrant.
"""

import os
import sys
import logging
import asyncio
from datetime import datetime
from typing import List, Dict, Optional, Any, Union
from urllib.parse import urljoin, urlparse
import json
import argparse
import time
import uuid

from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Import from modular services using relative imports
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from backend.config.settings import Config
from backend.services.ingestion import ContentExtractor
from backend.models.dataclasses import ContentPage, ContentChunk, EmbeddingVector, StoredVector, Query, RetrievedChunk, ValidationResult, RetrievalRequest, RetrievalResponse, ContentPageState
from backend.services.chunking import ContentChunker
from backend.services.embedding import EmbeddingGenerator
from backend.services.storage import VectorStorage
from backend.services.retrieval import EnhancedQdrantRetriever
from backend.services.validation import validate_retrieval_results, validate_semantic_relevance, validate_metadata_accuracy, validate_consistency
from backend.utils.helpers import setup_logging, retry_with_backoff, validate_embedding_dimensions
from backend.utils.validators import validate_chunk_integrity, validate_chunk_list_integrity

# -------------------- Safe Initialization --------------------
all_chunks: List[ContentChunk] = []
embeddings: List[EmbeddingVector] = []
stored_vectors: List[StoredVector] = []

# -------------------- Health Check --------------------
def test_qdrant_health(config: Config) -> Dict[str, Any]:
    try:
        from qdrant_client import QdrantClient
        start_time = time.time()
        client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
            prefer_grpc=False,
            timeout=config.request_timeout
        )
        collection_info = client.get_collection(config.qdrant_collection_name)
        is_healthy = True
        response_time = (time.time() - start_time) * 1000
        return {
            "service": "retrieval-service",
            "status": "healthy" if is_healthy else "unhealthy",
            "database_connection": "connected" if is_healthy else "disconnected",
            "qdrant_collection": config.qdrant_collection_name,
            "total_vectors": collection_info.points_count if is_healthy else 0,
            "last_heartbeat": datetime.now().isoformat(),
            "response_time_ms": response_time
        }
    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        return {
            "service": "retrieval-service",
            "status": "unhealthy",
            "database_connection": "disconnected",
            "qdrant_collection": config.qdrant_collection_name,
            "total_vectors": 0,
            "last_heartbeat": datetime.now().isoformat(),
            "response_time_ms": 0,
            "error": str(e)
        }

# -------------------- Main --------------------
def main():
    parser = argparse.ArgumentParser(description='Content Extraction, Embedding, and Retrieval Validation System')
    parser.add_argument('--urls', nargs='+', help='List of URLs to process')
    parser.add_argument('--sitemap', help='Sitemap URL to parse for URLs')
    parser.add_argument('--chunk-size', type=int, default=1000, help='Chunk size for content')
    parser.add_argument('--overlap-size', type=int, default=200, help='Overlap size for chunks')
    parser.add_argument('--validate', action='store_true', help='Run validation after processing')
    parser.add_argument('--query', help='Test query for retrieval validation')
    parser.add_argument('--queries', nargs='+', help='Multiple test queries for batch retrieval')
    parser.add_argument('--top-k', type=int, default=5, help='Number of results to return for retrieval')
    parser.add_argument('--similarity-threshold', type=float, default=0.5, help='Minimum similarity threshold')
    parser.add_argument('--test-retrieval', action='store_true', help='Run comprehensive retrieval validation tests')
    parser.add_argument('--num-tests', type=int, default=10, help='Number of tests to run for validation')
    parser.add_argument('--benchmark', action='store_true', help='Run performance benchmarking')
    parser.add_argument('--health', action='store_true', help='Check health of Qdrant connection')
    parser.add_argument('--validate-relevance', action='store_true', help='Run semantic relevance validation')
    parser.add_argument('--expected-topics', nargs='+', help='Expected topics for relevance validation')

    args = parser.parse_args()

    # Validate input arguments
    if args.chunk_size <= 0:
        logger.error("--chunk-size must be greater than 0")
        return
    if args.overlap_size < 0:
        logger.error("--overlap-size must be >= 0")
        return
    if args.top_k <= 0:
        logger.error("--top-k must be greater than 0")
        return
    if not 0 <= args.similarity_threshold <= 1:
        logger.error("--similarity-threshold must be between 0 and 1")
        return

    # Initialize configuration
    try:
        config = Config(chunk_size=args.chunk_size, overlap_size=args.overlap_size)
    except ValueError as e:
        logger.error(f"Configuration validation failed: {str(e)}")
        return

    logger.info("Content extraction, embedding, and retrieval validation system initialized")
    logger.info(f"Processing base URL: {config.book_base_url}")

    # -------------------- Health Check --------------------
    if args.health:
        logger.info("Performing health check...")
        health_info = test_qdrant_health(config)
        print(json.dumps(health_info, indent=2))
        return

    # -------------------- Retrieval Queries --------------------
    if args.query or args.queries:
        queries = [args.query] if args.query else args.queries
        for query_text in queries:
            logger.info(f"Processing retrieval query: {query_text}")
            try:
                # Use the EnhancedQdrantRetriever to retrieve chunks
                retriever = EnhancedQdrantRetriever(config)
                retrieved_chunks = retriever.retrieve_similar_chunks(
                    query_text=query_text,
                    top_k=args.top_k,
                    similarity_threshold=args.similarity_threshold
                )

                print(f"\nQuery: {query_text}")
                print(f"Retrieved {len(retrieved_chunks)} chunks")

                for i, chunk in enumerate(retrieved_chunks, 1):
                    print(f"{i}. [{chunk.similarity_score:.2f}] {chunk.metadata.get('url', 'N/A')} - {chunk.content[:100]}...")
            except Exception as e:
                print(f"Error retrieving results: {str(e)}")
        return

    # -------------------- Content Extraction --------------------
    extractor = ContentExtractor(config)
    content_pages: List[ContentPage] = []

    if args.sitemap:
        logger.info(f"Extracting content from sitemap: {args.sitemap}")
        try:
            import lxml  # Ensure XML parser is installed
            content_pages = extractor.extract_from_sitemap(args.sitemap)
        except Exception as e:
            logger.error(f"Sitemap extraction failed: {str(e)}")
    elif args.urls:
        logger.info(f"Extracting content from provided URLs: {args.urls}")
        for url in args.urls:
            content_pages.append(extractor.extract_content_from_url(url))
    else:
        logger.warning("No URLs, sitemap, or query provided.")

    # -------------------- Content Pipeline --------------------
    extracted_pages_count = len([cp for cp in content_pages if cp.state == ContentPageState.EXTRACTED])
    failed_pages_count = len([cp for cp in content_pages if cp.state == ContentPageState.FAILED])
    logger.info(f"Successfully extracted content from {extracted_pages_count} pages out of {len(content_pages)} total")
    if failed_pages_count > 0:
        logger.warning(f"Failed to extract content from {failed_pages_count} pages")

    # Initialize variables safely
    global all_chunks, embeddings, stored_vectors
    all_chunks = []
    embeddings = []
    stored_vectors = []

    # Only process pages that were successfully extracted
    successful_content_pages = [cp for cp in content_pages if cp.state == ContentPageState.EXTRACTED]

    if extracted_pages_count > 0:
        # Chunk the content
        chunker = ContentChunker(chunk_size=config.chunk_size, overlap_size=config.overlap_size)
        for content_page in successful_content_pages:
            if content_page.state == ContentPageState.EXTRACTED:
                chunks = chunker.chunk_content_page(content_page)
                all_chunks.extend(chunks)
        logger.info(f"Created {len(all_chunks)} chunks")

        # Generate embeddings
        if all_chunks:
            embedder = EmbeddingGenerator(config.cohere_api_key)
            embeddings = embedder.generate_embeddings(all_chunks)
            logger.info(f"Generated {len(embeddings)} embeddings")

            # Store vectors
            if embeddings and all_chunks:  # Make sure we have both embeddings and chunks
                try:
                    storage = VectorStorage(config.qdrant_url, config.qdrant_api_key, config.qdrant_collection_name)
                    stored_vectors = storage.store_embeddings(embeddings, all_chunks)
                    logger.info(f"Stored {len(stored_vectors)} vectors in Qdrant")

                    if args.validate:
                        logger.info("Running validation...")
                        print(f"Validation: Successfully stored {len(stored_vectors)} vectors in Qdrant")
                except Exception as e:
                    logger.error(f"Failed to store vectors in Qdrant: {str(e)}")
                    print(f"Error storing vectors: {str(e)}")
                    # Continue with the process even if storage fails
                    stored_vectors = []

    # -------------------- Final Summary --------------------
    print(
        f"System completed processing. "
        f"Successfully extracted {extracted_pages_count} pages, "
        f"failed to extract {failed_pages_count} pages, "
        f"created {len(all_chunks)} chunks, "
        f"generated {len(embeddings)} embeddings, "
        f"stored {len(stored_vectors)} vectors in Qdrant."
    )


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# """
# Content Extraction, Embedding Generation, and Vector Storage System

# This system extracts content from Docusaurus sites, chunks it for semantic retrieval,
# generates embeddings using Cohere, and stores vectors with metadata in Qdrant.
# """

# import os
# import sys
# import logging
# import asyncio
# from datetime import datetime
# from typing import List, Dict, Optional, Any, Union
# from urllib.parse import urljoin, urlparse
# import json
# import argparse
# import time
# import uuid


# from dotenv import load_dotenv

# # Load environment variables
# load_dotenv()

# # Initialize logging
# logging.basicConfig(
#     level=logging.INFO,
#     format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
# )
# logger = logging.getLogger(__name__)

# # Import from modular services
# from .config.settings import Config
# from .services.ingestion import ContentExtractor

# from .models.dataclasses import ContentPage, ContentChunk, EmbeddingVector, StoredVector, Query, RetrievedChunk, ValidationResult, RetrievalRequest, RetrievalResponse
# from .services.chunking import ContentChunker
# from .services.embedding import EmbeddingGenerator
# from .services.storage import VectorStorage
# from .services.retrieval import QdrantRetriever, simple_retrieve_chunks
# from .services.validation import validate_retrieval_results, validate_semantic_relevance, validate_metadata_accuracy, validate_consistency
# from .utils.helpers import setup_logging, retry_with_backoff, validate_embedding_dimensions
# from .utils.validators import validate_chunk_integrity, validate_chunk_list_integrity


# def test_qdrant_health(config: Config) -> Dict[str, Any]:
#     """
#     Test Qdrant health and return health check information.

#     Args:
#         config: Configuration object containing Qdrant connection details

#     Returns:
#         Dict containing health check information
#     """
#     try:
#         from qdrant_client import QdrantClient

#         start_time = time.time()
#         client = QdrantClient(
#             url=config.qdrant_url,
#             api_key=config.qdrant_api_key,
#             prefer_grpc=False,
#             timeout=config.request_timeout
#         )

#         # Test the connection by getting collection info
#         collection_info = client.get_collection(config.qdrant_collection_name)
#         is_healthy = True
#         response_time = (time.time() - start_time) * 1000  # Convert to milliseconds

#         return {
#             "service": "retrieval-service",
#             "status": "healthy" if is_healthy else "unhealthy",
#             "database_connection": "connected" if is_healthy else "disconnected",
#             "qdrant_collection": config.qdrant_collection_name,
#             "total_vectors": collection_info.points_count if is_healthy else 0,
#             "last_heartbeat": datetime.now().isoformat(),
#             "response_time_ms": response_time
#         }
#     except Exception as e:
#         logger.error(f"Health check failed: {str(e)}")
#         return {
#             "service": "retrieval-service",
#             "status": "unhealthy",
#             "database_connection": "disconnected",
#             "qdrant_collection": config.qdrant_collection_name,
#             "total_vectors": 0,
#             "last_heartbeat": datetime.now().isoformat(),
#             "response_time_ms": 0,
#             "error": str(e)
#         }


# def process_query_for_retrieval(config: Config, query_text: str) -> Query:
#     """
#     Process a query text into a Query object with embedding.

#     Args:
#         config: Configuration object containing API keys and settings
#         query_text: Text of the query

#     Returns:
#         Query object with text and generated embedding
#     """
#     try:
#         import cohere

#         cohere_client = cohere.Client(config.cohere_api_key)
#         response = cohere_client.embed(
#             texts=[query_text],
#             model=config.retrieval_model_name,
#             input_type="search_query"  # Using search_query for query embeddings
#         )

#         embedding = response.embeddings[0]
#         logger.info(f"Generated query embedding with {len(embedding)} dimensions")

#         # Validate embedding dimensions
#         if not validate_embedding_dimensions(embedding):
#             logger.warning(f"Embedding dimensions may not match stored vectors: {len(embedding)}")

#         query = Query(
#             text=query_text,
#             embedding=embedding,
#             created_at=datetime.now()
#         )

#         logger.info(f"Processed query with embedding of {len(embedding)} dimensions")
#         return query
#     except Exception as e:
#         logger.error(f"Failed to process query for retrieval: {str(e)}")
#         raise


# def perform_retrieval(config: Config, query_text: str, top_k: int = 5, similarity_threshold: float = 0.5) -> RetrievalResponse:
#     """
#     Perform complete retrieval process: process query, generate embedding, search Qdrant, return results.

#     Args:
#         config: Configuration object with API keys and settings
#         query_text: Text of the query to process
#         top_k: Number of top results to return
#         similarity_threshold: Minimum similarity threshold for results

#     Returns:
#         RetrievalResponse object with retrieved chunks and metadata
#     """
#     start_time = time.time()
#     try:
#         # Process the query to generate embedding
#         query = process_query_for_retrieval(config, query_text)

#         # Initialize the retriever
#         retriever = QdrantRetriever(config)

#         # Perform similarity search
#         retrieved_chunks = retriever.search_similar(
#             query_embedding=query.embedding,
#             top_k=top_k,
#             similarity_threshold=similarity_threshold
#         )

#         # Calculate query time
#         query_time_ms = (time.time() - start_time) * 1000

#         # Create retrieval response
#         response = RetrievalResponse(
#             request_id=str(uuid.uuid4()),
#             retrieved_chunks=retrieved_chunks,
#             query_time_ms=query_time_ms,
#             status="success",
#             total_results=len(retrieved_chunks)
#         )

#         logger.info(f"Retrieval completed in {query_time_ms:.2f}ms, returned {len(retrieved_chunks)} chunks")
#         return response

#     except Exception as e:
#         query_time_ms = (time.time() - start_time) * 1000
#         logger.error(f"Retrieval failed: {str(e)}")

#         # Return error response
#         response = RetrievalResponse(
#             request_id=str(uuid.uuid4()),
#             retrieved_chunks=[],
#             query_time_ms=query_time_ms,
#             status="error",
#             total_results=0,
#             error_message=str(e)
#         )

#         return response


# def main():
#     """Main entry point for the application."""
#     parser = argparse.ArgumentParser(description='Content Extraction, Embedding, and Retrieval Validation System')
#     parser.add_argument('--urls', nargs='+', help='List of URLs to process')
#     parser.add_argument('--sitemap', help='Sitemap URL to parse for URLs')
#     parser.add_argument('--chunk-size', type=int, default=1000, help='Chunk size for content')
#     parser.add_argument('--overlap-size', type=int, default=200, help='Overlap size for chunks')
#     parser.add_argument('--validate', action='store_true', help='Run validation after processing')
#     parser.add_argument('--query', help='Test query for retrieval validation')
#     parser.add_argument('--queries', nargs='+', help='Multiple test queries for batch retrieval')
#     parser.add_argument('--top-k', type=int, default=5, help='Number of results to return for retrieval')
#     parser.add_argument('--similarity-threshold', type=float, default=0.5, help='Minimum similarity threshold')
#     parser.add_argument('--test-retrieval', action='store_true', help='Run comprehensive retrieval validation tests')
#     parser.add_argument('--num-tests', type=int, default=10, help='Number of tests to run for validation')
#     parser.add_argument('--benchmark', action='store_true', help='Run performance benchmarking')
#     parser.add_argument('--health', action='store_true', help='Check health of Qdrant connection')
#     parser.add_argument('--validate-relevance', action='store_true', help='Run semantic relevance validation')
#     parser.add_argument('--expected-topics', nargs='+', help='Expected topics for relevance validation')

#     args = parser.parse_args()

#     # Validate input arguments
#     if args.chunk_size <= 0:
#         logger.error("--chunk-size must be greater than 0")
#         return

#     if args.overlap_size < 0:
#         logger.error("--overlap-size must be greater than or equal to 0")
#         return

#     if args.top_k <= 0:
#         logger.error("--top-k must be greater than 0")
#         return

#     if not 0 <= args.similarity_threshold <= 1:
#         logger.error("--similarity-threshold must be between 0 and 1")
#         return

#     # Initialize configuration
#     try:
#         config = Config(
#             chunk_size=args.chunk_size,
#             overlap_size=args.overlap_size
#         )
#     except ValueError as e:
#         logger.error(f"Configuration validation failed: {str(e)}")
#         return

#     logger.info("Content extraction, embedding, and retrieval validation system initialized")
#     logger.info(f"Processing base URL: {config.book_base_url}")

#     # Handle health check first if requested
#     if args.health:
#         logger.info("Performing health check...")
#         health_info = test_qdrant_health(config)
#         print(json.dumps(health_info, indent=2))
#         return

#     # Handle retrieval/query if specified
#     if args.query or args.queries:
#         # Set retrieval-specific config values
#         config.retrieval_top_k = args.top_k
#         config.retrieval_similarity_threshold = args.similarity_threshold

#         queries = [args.query] if args.query else args.queries
#         expected_topics = args.expected_topics if args.expected_topics else None

#         for query_text in queries:
#             logger.info(f"Processing retrieval query: {query_text}")

#             # Perform retrieval
#             retrieval_response = perform_retrieval(
#                 config,
#                 query_text,
#                 top_k=args.top_k,
#                 similarity_threshold=args.similarity_threshold
#             )

#             if retrieval_response.status == "success":
#                 print(f"\nQuery: {query_text}")
#                 print(f"Retrieved {retrieval_response.total_results} chunks in {retrieval_response.query_time_ms:.2f}ms")

#                 for i, chunk in enumerate(retrieval_response.retrieved_chunks, 1):
#                     print(f"{i}. [{chunk.similarity_score:.2f}] {chunk.metadata.get('url', 'N/A')} - {chunk.content[:100]}...")

#                 # Run validation if requested
#                 if args.validate_relevance:
#                     validation_result = validate_retrieval_results(
#                         query_text,
#                         retrieval_response.retrieved_chunks,
#                         config,
#                         expected_topics
#                     )

#                     print(f"Relevance Score: {validation_result.semantic_relevance_score:.2f}")
#                     print(f"Metadata Accuracy: {validation_result.metadata_accuracy:.2f}")
#                     print(f"Consistency Score: {validation_result.consistency_score:.2f}")
#                     print(f"Validation Time: {validation_result.latency_ms:.2f}ms")
#                     print(f"Validation: {'PASSED' if validation_result.semantic_relevance_score >= 0.8 else 'NEEDS REVIEW'} "
#                           f"({validation_result.semantic_relevance_score*100:.1f}% relevance, "
#                           f"{validation_result.metadata_accuracy*100:.1f}% metadata accuracy)")
#             else:
#                 print(f"Error retrieving results: {retrieval_response.error_message}")

#         return

#     # Handle comprehensive testing if requested
#     if args.test_retrieval:
#         logger.info(f"Running comprehensive retrieval tests with {args.num_tests} sample queries...")

#         # Sample test queries from different book topics
#         test_queries = [
#             "What are the core principles of RAG systems?",
#             "How does content chunking work?",
#             "Explain embedding techniques",
#             "What is semantic search?",
#             "How to validate retrieval results?",
#             "What are the benefits of vector databases?",
#             "Explain similarity scoring",
#             "How does Qdrant work?",
#             "What is Cohere embedding?",
#             "How to measure retrieval performance?"
#         ]

#         # Limit to the number of tests specified
#         test_queries = test_queries[:args.num_tests]

#         total_relevance = 0
#         total_accuracy = 0
#         successful_queries = 0

#         for i, query in enumerate(test_queries, 1):
#             print(f"\nTest {i}/{len(test_queries)}: {query}")

#             try:
#                 retrieval_response = perform_retrieval(config, query, top_k=args.top_k)

#                 if retrieval_response.status == "success":
#                     validation_result = validate_retrieval_results(query, retrieval_response.retrieved_chunks, config)

#                     print(f"  Results: {retrieval_response.total_results} chunks in {retrieval_response.query_time_ms:.2f}ms")
#                     print(f"  Relevance: {validation_result.semantic_relevance_score:.3f}")
#                     print(f"  Accuracy: {validation_result.metadata_accuracy:.3f}")
#                     print(f"  Consistency: {validation_result.consistency_score:.3f}")

#                     total_relevance += validation_result.semantic_relevance_score
#                     total_accuracy += validation_result.metadata_accuracy
#                     successful_queries += 1
#                 else:
#                     print(f"  Error: {retrieval_response.error_message}")
#             except Exception as e:
#                 print(f"  Exception: {str(e)}")

#         if successful_queries > 0:
#             avg_relevance = total_relevance / successful_queries
#             avg_accuracy = total_accuracy / successful_queries
#             print(f"\n--- Test Summary ---")
#             print(f"Successful queries: {successful_queries}/{len(test_queries)}")
#             print(f"Average relevance: {avg_relevance:.3f}")
#             print(f"Average metadata accuracy: {avg_accuracy:.3f}")
#         else:
#             print(f"\nAll retrieval tests failed!")

#         return

#     # Handle benchmarking if requested
#     if args.benchmark:
#         logger.info("Running performance benchmarking...")

#         # Sample queries for benchmarking
#         benchmark_queries = [
#             "What are the core principles?",
#             "Explain the concept",
#             "How does it work?",
#             "What are the benefits?",
#             "Show me examples"
#         ]

#         total_time = 0
#         successful_queries = 0

#         for query in benchmark_queries:
#             start_time = time.time()
#             try:
#                 retrieval_response = perform_retrieval(config, query, top_k=args.top_k)
#                 query_time = (time.time() - start_time) * 1000  # Convert to milliseconds

#                 if retrieval_response.status == "success":
#                     print(f"Query '{query}' took {query_time:.2f}ms and returned {retrieval_response.total_results} results")
#                     total_time += query_time
#                     successful_queries += 1
#                 else:
#                     print(f"Query '{query}' failed: {retrieval_response.error_message}")
#             except Exception as e:
#                 print(f"Query '{query}' exception: {str(e)}")

#         if successful_queries > 0:
#             avg_time = total_time / successful_queries
#             print(f"\n--- Benchmark Results ---")
#             print(f"Successful queries: {successful_queries}/{len(benchmark_queries)}")
#             print(f"Average query time: {avg_time:.2f}ms")
#             print(f"Target (<1 second): {'MET' if avg_time < 1000 else 'MISSED'}")
#         else:
#             print(f"\nAll benchmark queries failed!")

#         return

#     # Original content extraction and embedding pipeline
#     # Initialize content extractor
#     extractor = ContentExtractor(config)

#     content_pages = []

#     if args.sitemap:
#         logger.info(f"Extracting content from sitemap: {args.sitemap}")
#         content_pages = extractor.extract_from_sitemap(args.sitemap)
#     elif args.urls:
#         logger.info(f"Extracting content from provided URLs: {args.urls}")
#         for url in args.urls:
#             content_page = extractor.extract_content_from_url(url)
#             content_pages.append(content_page)
#     else:
#         logger.warning("No URLs, sitemap, or query provided. Use --urls, --sitemap, or --query to specify content to process.")

#     logger.info(f"Successfully extracted content from {len([cp for cp in content_pages if cp.state == ContentPage.ContentPageState.EXTRACTED])} pages out of {len(content_pages)} total")

#     # Process the extracted content through the full pipeline if we have successful extractions
#     if content_pages and any(cp.state == ContentPage.ContentPageState.EXTRACTED for cp in content_pages):
#         # Chunk the content
#         chunker = ContentChunker(chunk_size=config.chunk_size, overlap_size=config.overlap_size)
#         all_chunks = []
#         for content_page in content_pages:
#             if content_page.state == ContentPage.ContentPageState.EXTRACTED:
#                 chunks = chunker.chunk_content_page(content_page)
#                 all_chunks.extend(chunks)

#         logger.info(f"Created {len(all_chunks)} chunks from extracted content")

#         # Generate embeddings
#         if all_chunks:
#             embedder = EmbeddingGenerator(config.cohere_api_key)
#             embeddings = embedder.generate_embeddings(all_chunks)

#             logger.info(f"Generated {len(embeddings)} embeddings")

#             # Store in Qdrant
#             if embeddings:
#                 storage = VectorStorage(config.qdrant_url, config.qdrant_api_key, config.qdrant_collection_name)
#                 stored_vectors = storage.store_embeddings(embeddings, all_chunks)

#                 logger.info(f"Stored {len(stored_vectors)} vectors in Qdrant")

#                 # Run validation if requested
#                 if args.validate:
#                     logger.info("Running validation...")
#                     # For now, just print the number of stored vectors as a basic validation
#                     print(f"Validation: Successfully stored {len(stored_vectors)} vectors in Qdrant")

#                     if args.query:
#                         # Use the new retrieval functionality
#                         retrieval_response = perform_retrieval(config, args.query, top_k=args.top_k)

#                         if retrieval_response.status == "success":
#                             print(f"Retrieved {retrieval_response.total_results} chunks for query '{args.query}' in {retrieval_response.query_time_ms:.2f}ms")
#                         else:
#                             print(f"Failed to retrieve results for query '{args.query}': {retrieval_response.error_message}")

#     print(f"System completed processing. Extracted {len([cp for cp in content_pages if cp.state == ContentPage.ContentPageState.EXTRACTED])} pages, created {len(all_chunks)} chunks, generated {len(embeddings)} embeddings, and stored {len(stored_vectors)} vectors in Qdrant.")


# if __name__ == "__main__":
#     main()