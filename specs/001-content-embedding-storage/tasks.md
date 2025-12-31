# Tasks: Content Extraction, Embedding Generation, and Vector Storage

**Feature**: Content Extraction, Embedding Generation, and Vector Storage
**Branch**: `001-content-embedding-storage`
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Implementation Strategy

MVP scope: Complete User Story 1 (content extraction) as a standalone, testable system. This provides core value of extracting clean content from Docusaurus sites. Subsequent stories build on this foundation with chunking and embedding capabilities.

## Dependencies

User stories have sequential dependencies:
- US2 (Chunking) depends on US1 (Extraction) - needs extracted content as input
- US3 (Embedding & Storage) depends on US2 (Chunking) - needs content chunks as input

## Parallel Execution Examples

Per User Story 1:
- T001-T003: Project setup tasks can be done in parallel with research
- T006-P, T007-P, T008-P: Different components of content extraction can be developed in parallel

Per User Story 2:
- T015-P, T016-P: Different aspects of chunking algorithm can be developed in parallel

Per User Story 3:
- T022-P, T023-P: Embedding generation and Qdrant storage can be developed in parallel

## Phase 1: Setup

**Goal**: Initialize project structure and dependencies per implementation plan

- [X] T001 Create backend/ directory structure
- [X] T002 Create pyproject.toml with required dependencies (requests, beautifulsoup4, cohere, qdrant-client, python-dotenv)
- [X] T003 Create .env file template with environment variables (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, BOOK_BASE_URL)
- [X] T004 Create requirements.txt using uv export
- [X] T005 Install and verify dependencies with uv

## Phase 2: Foundational Components

**Goal**: Create foundational components that all user stories depend on

- [X] T006 Create main.py file structure with configuration loading from .env
- [X] T007 [P] Create ContentPage data class with validation per data-model.md
- [X] T008 [P] Create ContentChunk data class with validation per data-model.md
- [X] T009 [P] Create EmbeddingVector data class with validation per data-model.md
- [X] T010 [P] Create StoredVector data class with validation per data-model.md
- [X] T011 Create configuration class to manage settings and environment variables
- [X] T012 Create utility functions for logging and error handling

## Phase 3: [US1] Extract Clean Content from Book URLs

**Goal**: Implement content extraction from Docusaurus book URLs, filtering navigation elements and preserving document structure

**Independent Test Criteria**: System can successfully fetch content from sample book URLs and return clean text with headings preserved but navigation elements removed

**Acceptance Tests**:
- [X] T013 [US1] Test that URL returns clean content with headings preserved
- [X] T014 [US1] Test that navigation and footer elements are removed

**Implementation Tasks**:
- [X] T015 [P] [US1] Implement URL fetching with requests library and error handling
- [X] T016 [P] [US1] Implement HTML parsing with BeautifulSoup for Docusaurus content
- [X] T017 [P] [US1] Implement CSS selector logic to target Docusaurus-specific content areas (.markdown, article, [role="main"])
- [X] T018 [US1] Implement navigation/filtering logic to remove menus, footers, and irrelevant UI elements
- [X] T019 [US1] Implement heading extraction and preservation with position tracking
- [X] T020 [US1] Implement ContentPage creation with metadata (url, title, headings, extracted_at, source_format)
- [X] T021 [US1] Implement sitemap parsing for automatic URL discovery
- [X] T022 [US1] Add retry logic with exponential backoff for network requests
- [X] T023 [US1] Add rate limiting to respect website policies

## Phase 4: [US2] Chunk Content for Semantic Retrieval

**Goal**: Break down extracted content into appropriately sized chunks optimized for semantic retrieval with overlap preservation

**Independent Test Criteria**: System can take a full book chapter and split it into semantically meaningful chunks of appropriate size with proper overlap

**Acceptance Tests**:
- [X] T024 [US2] Test that long chapters are split into appropriate-sized chunks
- [X] T025 [US2] Test that overlap preserves context across boundaries

**Implementation Tasks**:
- [X] T026 [P] [US2] Implement basic chunking algorithm with configurable size (default 1000 chars)
- [X] T027 [P] [US2] Implement overlap logic with configurable size (default 200 chars)
- [X] T028 [US2] Implement sentence-boundary-aware chunking to respect text flow
- [X] T029 [US2] Implement heading context preservation within chunks
- [X] T030 [US2] Implement ContentChunk creation with metadata (source_page_url, chunk_index, overlap_with_next)
- [X] T031 [US2] Add validation to ensure chunks don't exceed embedding model limits
- [X] T032 [US2] Implement chunk boundary optimization to maintain semantic coherence

## Phase 5: [US3] Generate Embeddings and Store in Vector Database

**Goal**: Generate Cohere embeddings for content chunks and store in Qdrant with searchable metadata

**Independent Test Criteria**: System can take content chunks, generate embeddings using Cohere models, and store in Qdrant with metadata allowing retrieval

**Acceptance Tests**:
- [X] T033 [US3] Test that content chunks generate valid embeddings
- [X] T034 [US3] Test that embeddings are stored in Qdrant with proper metadata
- [X] T035 [US3] Test that similarity search returns relevant chunks for sample queries

**Implementation Tasks**:
- [X] T036 [P] [US3] Implement Cohere API client initialization with API key from environment
- [X] T037 [P] [US3] Implement Qdrant client initialization with URL and API key from environment
- [X] T038 [US3] Implement embedding generation using Cohere embed-multilingual-v3.0 model
- [X] T039 [US3] Implement Qdrant collection creation with proper vector dimensions (1024 for Cohere model)
- [X] T040 [US3] Implement vector storage with metadata (URL, chapter, section, title, chunk_index)
- [X] T041 [US3] Implement EmbeddingVector creation with model information and timestamps
- [X] T042 [US3] Implement StoredVector creation with complete payload per data-model.md
- [X] T043 [US3] Implement batch processing for efficient embedding and storage
- [X] T044 [US3] Implement similarity search functionality for validation
- [X] T045 [US3] Add rate limiting for Cohere API calls to avoid exceeding quotas

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Complete the system with validation, error handling, and command-line interface

- [X] T046 Implement comprehensive error handling and logging across all components
- [X] T047 Create command-line interface for main.py with options for URLs, sitemap, chunk size, validation
- [X] T048 Implement progress tracking and statistics reporting
- [X] T049 Add validation function to run sample search and verify pipeline functionality
- [X] T050 Write comprehensive documentation for setup and usage
- [X] T051 Add unit tests for core functionality
- [X] T052 Perform end-to-end testing with actual Docusaurus site
- [X] T053 Optimize performance for large content sets
- [X] T054 Update quickstart guide with actual implementation details