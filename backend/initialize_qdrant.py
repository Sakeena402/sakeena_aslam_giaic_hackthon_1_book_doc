#!/usr/bin/env python3
"""
Script to initialize the Qdrant collection for the retrieval system.
"""

import os
import sys
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

def initialize_qdrant_collection():
    """Initialize the Qdrant collection with proper configuration."""

    # Get configuration from environment variables
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

    if not qdrant_url or not qdrant_api_key:
        print("Error: QDRANT_URL and QDRANT_API_KEY must be set in environment variables")
        return False

    try:
        # Create Qdrant client
        client = QdrantClient(
            url=qdrant_url,
            api_key=qdrant_api_key,
            prefer_grpc=False,
            timeout=30
        )

        # Check if collection already exists
        try:
            collection_info = client.get_collection(collection_name)
            print(f"Collection '{collection_name}' already exists with {collection_info.points_count} points")
            return True
        except:
            # Collection doesn't exist, create it
            print(f"Creating collection '{collection_name}'...")

            # Create collection with 1024-dimensional vectors (for Cohere embed-multilingual-v3.0)
            client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(size=1024, distance=Distance.COSINE),  # 1024 is the dimension for Cohere v3.0
            )

            print(f"Successfully created collection '{collection_name}' with 1024-dimensional vectors")
            return True

    except Exception as e:
        print(f"Error initializing Qdrant collection: {str(e)}")
        return False

if __name__ == "__main__":
    print("Initializing Qdrant collection for retrieval system...")
    success = initialize_qdrant_collection()
    if success:
        print("Qdrant collection initialized successfully!")
        sys.exit(0)
    else:
        print("Failed to initialize Qdrant collection.")
        sys.exit(1)