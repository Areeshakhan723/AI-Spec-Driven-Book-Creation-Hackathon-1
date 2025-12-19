<!--
Sync Impact Report:
- Version change: 1.0.0 → 1.1.0
- Modified principles: Added 6 principles with specific project focus
- Added sections: Core Principles, Key Standards, Constraints, Success Criteria
- Removed sections: None
- Templates requiring updates: N/A
- Follow-up TODOs: None
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First Workflow
Spec-first workflow using Spec-Kit Plus: All features must be defined in specifications before implementation begins; Specifications serve as the single source of truth for requirements and acceptance criteria; All changes to functionality require spec updates first.

### Technical Accuracy
Technical accuracy from official sources: All information and code examples must be verified against official documentation; No hallucinated responses or fabricated information; Citations and references to authoritative sources required for technical claims.

### Developer-Focused Writing
Clear, developer-focused writing: Documentation must be accessible to developers of varying skill levels; Examples should be practical and runnable; Explanations should prioritize clarity over cleverness.

### Reproducible Setup and Deployment
Reproducible setup and deployment: All environments must be replicable from documentation alone; Containerized deployments preferred; Configuration management follows infrastructure-as-code principles.

### GitHub-Based Source Control
GitHub-based source control: All code and documentation stored in GitHub repositories; Pull request workflow with code reviews required for all changes; Branch protection rules enforced.

### No Hallucinated Responses
No hallucinated responses: The RAG chatbot must only respond based on book content or user-selected text; Strict grounding in provided documents required; When uncertain, the system should indicate lack of knowledge rather than generating content.

## Key Standards

### Book Platform
Book written with Docusaurus and deployed on GitHub Pages: Documentation platform uses Docusaurus for static site generation; Deployment occurs via GitHub Actions to GitHub Pages; Search functionality integrated for easy navigation.

### RAG Chatbot
RAG chatbot grounded only in book content or user-selected text: Retrieval-Augmented Generation system connects to document store; Responses must cite source material; Chat interface embedded seamlessly in documentation pages.

### Technology Stack
Technology stack: OpenAI Agents / ChatKit for conversational interface; FastAPI for backend services; Neon Postgres for relational data storage; Qdrant Cloud for vector embeddings and similarity search.

### Code Quality
Runnable, well-documented code: All code examples provided must be tested and functional; Comprehensive documentation for setup and usage; Clear separation of concerns in architecture.

## Constraints

### Source Control
GitHub-based source control: All development occurs in GitHub repositories; Collaboration follows established branching and merging practices; Issue tracking integrated with development workflow.

### Response Integrity
No hallucinated responses: System must strictly adhere to provided content; No generation of information outside of source materials; Clear indication when information is not available in sources.

### End-to-End Reproducibility
End-to-end reproducibility: Entire system must be deployable from source code and configuration; Environment variables and secrets managed securely; Build and deployment processes documented and automated.

## Success Criteria

### Live Deployment
Live book on GitHub Pages: Documentation accessible at published URL; Responsive design working across devices; Search and navigation functioning properly.

### Functional Chatbot
Fully functional embedded RAG chatbot: Real-time responses based on book content; Proper citation of sources in responses; Smooth integration with documentation interface.

### Specification Compliance
All specs implemented via Spec-Kit Plus: Features developed according to defined specifications; Acceptance criteria validated; Test coverage meeting defined standards.

## Governance

This constitution governs all aspects of the AI/Spec-Driven Book with Embedded RAG Chatbot project. All development activities, code reviews, and project decisions must comply with these principles. Amendments to this constitution require explicit approval from project stakeholders and must be documented with clear rationale. The constitution supersedes any conflicting practices or procedures.

**Version**: 1.1.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
