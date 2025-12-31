"""
Configuration and fixtures for tests.
"""

import pytest
import sys
from pathlib import Path

# Add the backend directory to the Python path
backend_path = Path(__file__).parent.parent
sys.path.insert(0, str(backend_path))


@pytest.fixture(autouse=True)
def setup_test_environment():
    """
    Setup test environment automatically for all tests.
    """
    # Set up any necessary test configurations
    import os
    os.environ.setdefault('COHERE_API_KEY', 'test-cohere-key')
    os.environ.setdefault('QDRANT_URL', 'http://localhost:6333')
    os.environ.setdefault('QDRANT_API_KEY', 'test-qdrant-key')
    os.environ.setdefault('BOOK_BASE_URL', 'https://example.com')

    yield

    # Cleanup after tests if needed
    pass


def pytest_configure(config):
    """
    Configure pytest.
    """
    config.addinivalue_line(
        "markers", "integration: mark test as integration test"
    )
    config.addinivalue_line(
        "markers", "unit: mark test as unit test"
    )


def pytest_collection_modifyitems(config, items):
    """
    Modify test items during collection.
    """
    # Add markers based on test file location
    for item in items:
        if "integration" in item.nodeid:
            item.add_marker(pytest.mark.integration)
        else:
            item.add_marker(pytest.mark.unit)