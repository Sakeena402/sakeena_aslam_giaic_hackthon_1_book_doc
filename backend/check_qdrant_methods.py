#!/usr/bin/env python3
"""
Script to check available methods on the Qdrant client.
"""

def check_qdrant_methods():
    """Check what methods are available on the Qdrant client."""
    from qdrant_client import QdrantClient

    # Create a mock client to inspect available methods
    # We won't actually connect, just need to see the methods
    client = QdrantClient(":memory:")  # In-memory for testing

    # Get all methods and attributes
    all_attrs = dir(client)

    # Filter for search-related methods
    search_methods = [attr for attr in all_attrs if 'search' in attr.lower()]

    print("Available search methods on QdrantClient:")
    for method in search_methods:
        print(f"  - {method}")

    print("\nAll methods containing 'search' or 'find':")
    search_like = [attr for attr in all_attrs if any(keyword in attr.lower() for keyword in ['search', 'find', 'retrieve', 'query'])]
    for method in search_like:
        print(f"  - {method}")

if __name__ == "__main__":
    check_qdrant_methods()