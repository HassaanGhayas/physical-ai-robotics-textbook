<!--
Sync Impact Report:
- Version change: N/A → 1.0.0
- Added sections: All principles and sections based on Physical AI & Humanoid Robotics project
- Templates requiring updates: N/A (initial constitution)
- Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Book Constitution

## Core Principles

### Spec-Driven Development
All development follows the Spec-Kit Plus methodology using Claude Code and Spec-Kit Plus for AI/Spec-driven book creation, ensuring structured and testable deliverables.

### Modular Architecture
The system is built with modular components: Docusaurus for book creation, RAG chatbot with Cohere Models, OpenAI Agents SDK, FastAPI, Neon Serverless Postgres, and Qdrant Cloud for separate concerns.

### AI Integration First
AI capabilities are integrated from the start: RAG chatbot for content queries, OpenAI Agents SDK for intelligence, and Cohere Models for advanced language understanding.

### Authentication & Personalization
User authentication implemented with Better Auth for secure access, with personalized features like Urdu translation capability for signed-in users.

### Performance & Scalability
Designed for performance with serverless Neon Postgres database and Qdrant Cloud Free Tier for scalable vector search, ensuring fast response times for the RAG system.

### Multi-language Support
Built-in Urdu translation feature for signed-in users, enabling accessibility for diverse audiences through a button-based translation system in the navbar.

## Technology Stack Requirements

Technology stack: Docusaurus for documentation, GitHub Pages for deployment, Cohere Models for RAG, OpenAI Agents SDK for AI capabilities, FastAPI for backend services, Neon Serverless Postgres for database, Qdrant Cloud for vector storage, Better Auth for authentication.

## Development Workflow

Development follows the sequence: Writing Book content → UI Customization (shadcn, lottie animations, clean theme) → UX Customization → GitHub Push → GitHub Pages Deployment → RAG Chatbot Development → Authentication Implementation → Testing.

## Governance

All changes must follow the Spec-Kit Plus methodology. Each feature must be implemented as per the outlined approach: Book Writing → UI/UX Customization → Deployment → Chatbot → Authentication → Testing. Compliance with Claude Code best practices required.

**Version**: 1.0.0 | **Ratified**: 2025-12-11 | **Last Amended**: 2025-12-11