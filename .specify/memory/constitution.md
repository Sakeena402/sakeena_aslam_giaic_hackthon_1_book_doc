<!--
Sync Impact Report:
- Version change: N/A → 1.0.0 (initial constitution)
- Added sections: All principles and governance sections
- Templates requiring updates: N/A (initial creation)
- Follow-up TODOs: None
-->
# AI/Spec-Driven Unified Book with Integrated RAG Chatbot Constitution

## Core Principles

### I. Spec-Driven Development
All implementation follows written specifications. Every feature, component, and functionality must have a clear, written specification before implementation begins. Code must align precisely with the defined spec, with any deviations requiring spec updates first.

### II. Accuracy and Consistency
Maintain accuracy and consistency across written content and chatbot responses. All content and AI responses must be factually correct and consistent with the book's established knowledge. The RAG chatbot must ground all responses in retrieved content only, avoiding hallucinations.

### III. Clarity and Accessibility
Ensure clarity for developers, students, and AI practitioners. All documentation, code, and UI elements must be clear and accessible to the target audience. Content should be structured with clear objectives, scope definitions, and navigable organization.

### IV. Reusability and Maintainability
Design for reusability and maintainability of specs and code. Components, specifications, and code modules should be designed for reuse where appropriate. Clear separation of concerns between content, retrieval, application, and UI layers is mandatory.

### V. Minimal and Modern UI/UX
Implement minimal, modern, and accessible UI/UX design. The interface should follow modern design principles with clean layout, generous white space, neutral color palette, light/dark mode support, and responsive design for all device types.

### VI. Seamless Integration
Achieve seamless integration between documentation and AI features. The RAG chatbot must be smoothly embedded in the book UI with non-intrusive interaction patterns, supporting both full-book queries and user-selected text queries.

## Technology Standards

All book content must be generated or refined through specs. Each chapter must have a clear objective and scope. The RAG chatbot responses must be grounded in retrieved data only with no response relying on unstated assumptions or external knowledge. The system must maintain clear separation between content layer (book), retrieval layer (Qdrant), application layer (FastAPI), and UI layer (Docusaurus frontend).

## Development Workflow

Deployment platform is GitHub Pages with backend using FastAPI (API-only, no server-side rendering). Database uses Neon Serverless Postgres for metadata and chat history. Vector DB uses Qdrant Cloud Free Tier for vector storage. OpenAI usage must be optimized to reduce cost. The frontend must be stateless with all intelligence handled via APIs.

## Governance

This constitution supersedes all other development practices and guidelines. All implementations must comply with these principles. Any amendments to this constitution require formal documentation, team approval, and migration planning. All PRs and reviews must verify constitutional compliance. Complexity must be justified against these core principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-25 | **Last Amended**: 2025-12-25
