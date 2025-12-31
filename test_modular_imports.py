#!/usr/bin/env python3
"""
Test script to verify that each module can be imported independently.
"""

import sys
import os

# Add the current directory to the Python path to allow imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

def test_imports():
    """Test that all modules can be imported without circular dependencies."""
    print("Testing module imports...")

    try:
        # Test config import
        from backend.config.settings import Config
        print("[SUCCESS] Config module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import Config module: {e}")
        return False

    try:
        # Test models import
        from backend.models.dataclasses import ContentPage, ContentChunk, EmbeddingVector, StoredVector, Query, RetrievedChunk, ValidationResult, RetrievalRequest, RetrievalResponse
        print("[SUCCESS] Data models imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import data models: {e}")
        return False

    try:
        # Test ingestion import
        from backend.services.ingestion import ContentExtractor
        print("[SUCCESS] Ingestion module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import ingestion module: {e}")
        return False

    try:
        # Test chunking import
        from backend.services.chunking import ContentChunker
        print("[SUCCESS] Chunking module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import chunking module: {e}")
        return False

    try:
        # Test embedding import
        from backend.services.embedding import EmbeddingGenerator
        print("[SUCCESS] Embedding module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import embedding module: {e}")
        return False

    try:
        # Test storage import
        from backend.services.storage import VectorStorage
        print("[SUCCESS] Storage module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import storage module: {e}")
        return False

    try:
        # Test retrieval import
        from backend.services.retrieval import QdrantRetriever
        print("[SUCCESS] Retrieval module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import retrieval module: {e}")
        return False

    try:
        # Test validation import
        from backend.services.validation import validate_retrieval_results
        print("[SUCCESS] Validation module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import validation module: {e}")
        return False

    try:
        # Test utils import
        from backend.utils.helpers import setup_logging
        print("[SUCCESS] Helpers module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import helpers module: {e}")
        return False

    try:
        # Test validators import
        from backend.utils.validators import validate_chunk_integrity
        print("[SUCCESS] Validators module imported successfully")
    except Exception as e:
        print(f"[ERROR] Failed to import validators module: {e}")
        return False

    print("\n[SUCCESS] All modules imported successfully without circular dependencies!")
    return True


if __name__ == "__main__":
    success = test_imports()
    if success:
        print("\n[SUCCESS] Import test passed - all modules can be imported independently!")
    else:
        print("\n[ERROR] Import test failed - there are circular dependencies or import issues!")
        exit(1)