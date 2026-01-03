# Quickstart: Agent-Based RAG Integration

## Setup and Installation

### Prerequisites
- Python 3.11+
- OpenAI API key
- Cohere API key
- Qdrant vector database instance
- Existing content already ingested in Qdrant (from Specs 01 & 02)

### Environment Configuration
```bash
# Copy environment template
cp .env.example .env

# Update .env with your API keys and Qdrant configuration
OPENAI_API_KEY=your_openai_api_key
COHERE_API_KEY=your_cohere_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_COLLECTION_NAME=your_collection_name
BOOK_BASE_URL=your_book_base_url
```

### Install Dependencies
```bash
pip install openai cohere qdrant-client python-dotenv
```

## Basic Usage

### Initialize the Agent
```python
from backend.agent import RAGAgent

# Initialize the agent with configuration
agent = RAGAgent.from_config()

# Process a query
response = agent.process_query("What is ROS 2?")
print(response.response_text)
```

### Global Queries (Entire Book)
```python
# Query across the entire book content
response = agent.process_query(
    query_text="Explain the concept of the robotic nervous system",
    query_type="global"
)

# Access grounding chunks
for chunk in response.grounding_chunks:
    print(f"Source: {chunk.source_url}")
    print(f"Content: {chunk.content[:200]}...")
```

### Section-Specific Queries
```python
# Query only within a specific section
response = agent.process_query(
    query_text="How does Isaac Sim differ from Isaac ROS?",
    query_type="section_specific",
    section_filter="isaac"
)

# The agent will only retrieve content from Isaac-related chapters
```

### Advanced Query Options
```python
# With custom parameters
response = agent.process_query(
    query_text="What are the benefits of using VLA systems?",
    top_k=5,  # Retrieve 5 chunks instead of default
    similarity_threshold=0.6,  # Adjust similarity threshold
    query_type="global"
)
```

## Testing the Agent

### Run Basic Tests
```bash
# Test global queries
python -c "from backend.agent import test_global_query; test_global_query()"

# Test section-specific queries
python -c "from backend.agent import test_section_query; test_section_query()"

# Run full integration tests
pytest tests/test_agent_integration.py
```

### Validation Checks
```python
# Verify grounding quality
from backend.agent import validate_grounding

response = agent.process_query("What is NVIDIA Isaac?")
is_validly_grounded = validate_grounding(response)
print(f"Response is properly grounded: {is_validly_grounded}")
```

## Development Workflow

### 1. Create the Agent Module
```bash
# Create the main agent file
touch backend/agent.py
```

### 2. Implement Core Classes
Start with the basic structure in `backend/agent.py`:
- RAGAgent class for the main agent logic
- AgentQuery and AgentResponse data models
- Integration with existing retrieval services

### 3. Connect to Existing Services
The agent should utilize:
- `backend.services.retrieval.QdrantRetriever` for content retrieval
- `backend.config.settings.Config` for configuration
- `backend.models.dataclasses` for data models

### 4. Implement Query Processing
```python
def process_query(self, query_text, query_type="global", section_filter=None):
    # 1. Transform query to embedding
    # 2. Retrieve relevant chunks from Qdrant
    # 3. Generate grounded response using OpenAI API
    # 4. Validate grounding and return response
```

## Key Integration Points

### With Existing Retrieval Pipeline
- Leverage `simple_retrieve_chunks` function from `retrieval.py`
- Use existing embedding and chunking configurations
- Integrate with current Qdrant collection structure

### With MCP Server
- Ensure agent is compatible with existing MCP server structure
- Follow existing patterns for backend service integration
- Maintain consistency with current API approaches

### Logging and Debugging
- Log all retrieved chunks for verification
- Track query-response pairs for analysis
- Monitor grounding quality metrics

## Troubleshooting

### Common Issues
- **Rate Limits**: Implement proper backoff strategies for API calls
- **Empty Results**: Check that content was properly ingested in Qdrant
- **Poor Grounding**: Verify similarity thresholds and retrieval parameters

### Debugging Commands
```bash
# Check Qdrant connection
python -m backend.main --health

# Test retrieval directly
python -m backend.main --query "test query" --top-k 3

# Verify agent initialization
python -c "from backend.agent import RAGAgent; agent = RAGAgent.from_config(); print('Agent initialized successfully')"
```