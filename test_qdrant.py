import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

# Load environment variables
load_dotenv()

# Get the configuration
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

print(f"QDRANT_URL: {qdrant_url}")
print(f"Collection name: {collection_name}")

try:
    # Create Qdrant client
    client = QdrantClient(
        url=qdrant_url,
        api_key=qdrant_api_key,
        prefer_grpc=False,
        timeout=10
    )
    print("Qdrant client created successfully")

    # Try to get collection info
    collection_info = client.get_collection(collection_name)
    print(f"Successfully connected to collection: {collection_name}")
    print(f"Collection points count: {collection_info.points_count}")

except Exception as e:
    print(f"Error connecting to Qdrant: {e}")
    import traceback
    traceback.print_exc()