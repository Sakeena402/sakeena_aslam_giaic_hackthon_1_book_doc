# Research: Content Extraction, Embedding Generation, and Vector Storage

## Overview
This research document addresses the technical requirements for implementing content extraction from Docusaurus sites, text chunking, embedding generation with Cohere, and storage in Qdrant vector database.

## Decision: Use Python with specific libraries for content extraction and processing
**Rationale**: Python provides excellent libraries for web scraping, text processing, and vector databases. The combination of requests/BeautifulSoup for extraction, custom chunking logic, Cohere for embeddings, and Qdrant client for storage provides a robust solution.

**Alternatives considered**:
- Node.js with Puppeteer/Playwright: More complex for text extraction
- Go: Less mature ecosystem for embeddings and vector databases
- Direct API integration: Not applicable for web scraping approach

## Decision: Content extraction approach using requests and BeautifulSoup
**Rationale**: The requests library combined with BeautifulSoup provides reliable HTML parsing capabilities. For Docusaurus sites, we can extract clean content by targeting specific CSS selectors that contain the main content while avoiding navigation elements.

**Implementation approach**:
- Use requests to fetch URLs
- Use BeautifulSoup to parse HTML
- Target Docusaurus-specific CSS classes like `.markdown` or `article` for content extraction
- Remove navigation, footer, and sidebar elements using CSS selectors

## Decision: Text chunking strategy with overlap
**Rationale**: Proper chunking is critical for semantic search quality. Using a sliding window approach with overlap preserves context across chunk boundaries.

**Implementation approach**:
- Chunk size: 1000-2000 characters (or 200-500 tokens) to balance context and precision
- Overlap: 200-400 characters to maintain context across boundaries
- Respect sentence boundaries where possible
- Preserve section headings with content when possible

## Decision: Cohere embedding model selection
**Rationale**: Cohere provides high-quality embeddings optimized for semantic search. The embed-multilingual-v3.0 model is recommended for technical content like documentation.

**Implementation approach**:
- Use Cohere's embed-multilingual-v3.0 model with input_type="search_document"
- Batch requests to optimize API usage
- Handle rate limiting and errors gracefully

## Decision: Qdrant vector database integration
**Rationale**: Qdrant provides a robust vector database solution with good Python client support. The cloud free tier meets the requirements for this project.

**Implementation approach**:
- Create a collection with appropriate vector dimensions (matching Cohere model)
- Store metadata including URL, chapter, and section information
- Use payload filtering for metadata-based searches
- Implement proper error handling for API interactions

## Decision: Project structure with single main.py file
**Rationale**: The requirement specifically calls for a single main.py file containing all functionality, with a backend/ folder structure.

**Implementation approach**:
- Organize code in main.py with clear function separation
- Use classes for different components (Extractor, Chunker, Embedder, Storage)
- Include command-line interface for execution
- Include validation functions to test the complete pipeline

## Decision: UV package manager setup
**Rationale**: UV is a fast Python package manager that works well for this type of project.

**Implementation approach**:
- Create pyproject.toml with required dependencies
- Use uv to manage dependencies and virtual environment
- Include proper development dependencies for testing

## Key Unknowns Resolved

### URL Structure for Docusaurus Site
- **Unknown**: How to identify the URLs to crawl from the deployed Docusaurus site
- **Resolution**: Will need to either parse the sitemap.xml, analyze the navigation structure, or accept URLs as input. For MVP, we'll implement sitemap parsing and provide manual URL list as fallback.

### Content Structure Extraction
- **Unknown**: Specific CSS selectors for Docusaurus content extraction
- **Resolution**: Will use common Docusaurus selectors like `.markdown`, `article`, or `[role="main"]` and make them configurable.

### Qdrant Collection Configuration
- **Unknown**: Optimal vector dimensions and configuration for Cohere embeddings
- **Resolution**: Cohere embed-multilingual-v3.0 produces 1024-dimension vectors, so we'll configure Qdrant accordingly.

### Error Handling Strategy
- **Unknown**: How to handle partial failures during the pipeline
- **Resolution**: Implement graceful degradation with detailed logging, continue processing other items when individual items fail.

## Technical Requirements Validation

### Functional Requirements Met
- FR-001: URL crawling - Implemented via requests and sitemap parsing
- FR-002: Clean text extraction - Implemented via BeautifulSoup with CSS selectors
- FR-003: Document structure preservation - Implemented by preserving headings with content
- FR-004: Content chunking - Implemented with configurable size and overlap
- FR-005: Overlap with context preservation - Implemented with sliding window approach
- FR-006: Cohere embeddings - Implemented via Cohere API client
- FR-007: Qdrant storage - Implemented via Qdrant client
- FR-008: Metadata storage - Implemented with URL, chapter, section in payload
- FR-009: Error handling - Implemented with try-catch and logging
- FR-10: Semantic coherence - Implemented by respecting text boundaries
- FR-011: Processing validation - Implemented with progress tracking
- FR-012: Traceability - Implemented with metadata storage
- FR-013: Large page handling - Implemented with pre-chunking split
- FR-014: Metadata consistency - Implemented with schema validation

### Success Criteria Validation
- SC-001: Page fetching - Validated by progress tracking and error logging
- SC-002: Structure preservation - Validated by testing with sample Docusaurus sites
- SC-003: Consistent chunking - Validated by configurable parameters
- SC-004: Embedding generation - Validated by API response handling
- SC-005: Metadata storage - Validated by payload schema
- SC-006: Similarity search - Validated by implementing search validation function
- SC-007: Pipeline performance - Validated by timing measurements
- SC-008: Traceability - Validated by metadata preservation

## Risk Assessment

### High Risk
- Network reliability: Web scraping may encounter timeouts or blocks
- Rate limiting: Cohere and Qdrant APIs may have rate limits
- Content structure changes: Docusaurus sites may change CSS classes

### Medium Risk
- Large content pages: May exceed embedding model limits
- Authentication: Some Docusaurus sites may require authentication
- Dynamic content: JavaScript-rendered content may not be captured with requests/BeautifulSoup

### Mitigation Strategies
- Implement retry logic with exponential backoff
- Use appropriate request headers to avoid blocking
- Implement fallback selectors for content extraction
- Add support for JavaScript rendering if needed (e.g., using Playwright as fallback)
- Implement rate limiting awareness and pacing