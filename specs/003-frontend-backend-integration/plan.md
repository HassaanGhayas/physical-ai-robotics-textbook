# Implementation Plan: Frontend-Backend Integration for RAG Chatbot

**Branch**: `003-frontend-backend-integration` | **Date**: 2025-12-17 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-frontend-backend-integration/spec.md`

## Summary

This plan outlines the implementation of a React-based chat interface integrated with the Docusaurus documentation site that communicates with the FastAPI RAG Agent backend. The integration enables users to ask questions about the Physical AI & Humanoid Robotics knowledge base and receive AI-generated answers with source citations.

**Primary Requirement**: Establish seamless communication between frontend and backend for conversational AI interactions
**Technical Approach**: React Context + axios API client + Docusaurus plugin architecture

## Technical Context

**Language/Version**: TypeScript 5.x, React 18.x (via Docusaurus)
**Primary Dependencies**: Docusaurus 3.x, Axios 1.6+, React 18.x
**Storage**: Session storage (optional for chat history persistence)
**Testing**: React Testing Library, Vitest
**Target Platform**: Modern web browsers (Chrome, Firefox, Safari, Edge), responsive mobile support
**Project Type**: Web application (frontend only, integrates with existing backend)
**Performance Goals**:
- Initial load: Chat component lazy-loaded, minimal bundle impact
- Response time: Display loading state within 100ms, full response within 5s for 95% of queries
- Re-render optimization: React.memo for message components

**Constraints**:
- Must work within Docusaurus architecture and theming
- CORS properly configured on backend
- No server-side rendering complications
- Accessibility: WCAG AA compliance

**Scale/Scope**:
- Single chat interface (modal/sidebar)
- Up to 50 concurrent users initially
- Support for 50 messages per session
- 5 source citations per response

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Spec-Driven Development
- **Status**: PASS
- **Evidence**: Complete feature specification exists at `specs/003-frontend-backend-integration/spec.md`
- **Verification**: Spec includes user scenarios, acceptance criteria, requirements, and success metrics

### ✅ Modular Architecture
- **Status**: PASS
- **Evidence**:
  - Frontend chat component is self-contained and can be enabled/disabled via config
  - Backend API already modular (separate RAG Agent service)
  - Clear separation of concerns: UI components, API client, state management
- **Verification**: Component tree is modular with ChatBot → ChatInterface → MessageList/InputBar/SourceList

### ✅ AI Integration First
- **Status**: PASS
- **Evidence**:
  - Integration with existing RAG Agent (OpenAI Agents SDK + Cohere + Qdrant)
  - Chat interface specifically designed for AI-generated responses with source attribution
- **Verification**: API contract supports query → embedding → retrieval → generation workflow

### ✅ Authentication & Personalization
- **Status**: NOT APPLICABLE (Out of Scope)
- **Evidence**: Per spec section "Out of Scope", user authentication is not included in v1
- **Justification**: Anonymous chat access allows immediate user engagement without friction
- **Future**: Can be added in v2 for personalized chat history

### ✅ Performance & Scalability
- **Status**: PASS
- **Evidence**:
  - Lazy loading of chat component reduces initial bundle
  - Backend already uses serverless Neon Postgres and Qdrant Cloud
  - Client-side optimizations: React.memo, code splitting, debouncing
- **Verification**: Performance targets defined: <100ms feedback, <5s response time for 95% of queries

### ✅ Multi-language Support
- **Status**: NOT APPLICABLE (Out of Scope)
- **Evidence**: Per spec "Assumptions", queries are English-only for v1
- **Justification**: Backend models (OpenAI, Cohere) are English-first; multilingual support requires translation layer
- **Future**: Can be added with translation service integration

### Constitution Summary

**Overall Assessment**: PASS with justifications

The implementation aligns with all applicable constitution principles. The two non-applicable principles (Authentication and Multi-language) are explicitly scoped out in the feature specification with clear rationale. No violations require justification in the Complexity Tracking section.

## Project Structure

### Documentation (this feature)

```text
specs/003-frontend-backend-integration/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification (existing)
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (data entities)
├── quickstart.md        # Phase 1 output (implementation guide)
├── contracts/           # Phase 1 output (API contracts)
│   ├── api-contract.yaml         # OpenAPI specification
│   └── frontend-api-client.ts    # TypeScript client interface
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus + React)
src/
├── components/
│   └── ChatBot/
│       ├── ChatBot.tsx           # Main container with toggle
│       ├── ChatInterface.tsx     # Chat UI modal/sidebar
│       ├── ChatContext.tsx       # React Context + useReducer
│       ├── MessageList.tsx       # Message rendering
│       ├── InputBar.tsx          # Query input
│       ├── SourceList.tsx        # Source citations
│       ├── ErrorBoundary.tsx     # Error handling
│       ├── LoadingIndicator.tsx  # Loading states
│       ├── styles.module.css     # Component styles
│       └── __tests__/
│           ├── ChatBot.test.tsx
│           ├── ChatInterface.test.tsx
│           └── MessageList.test.tsx
├── lib/
│   └── api/
│       ├── client.ts             # Axios API client implementation
│       ├── types.ts              # API request/response types
│       └── validators.ts         # Request/response validation
├── types/
│   └── chat.ts                   # Chat data models
└── theme/
    └── Root.tsx                  # Docusaurus root wrapper (add ChatBot)

# Backend (existing FastAPI service)
backend/
├── main.py                       # Update CORS configuration
└── (no other changes required)

# Configuration
.env.local                        # Environment variables (not committed)
.env.example                      # Example env file (committed)
docusaurus.config.js              # Add customFields for API config
package.json                      # Add axios dependency
```

**Structure Decision**: Web application structure selected because:
- Frontend is a Docusaurus site (React-based documentation framework)
- Backend is a separate FastAPI service (already exists)
- Clear separation allows independent deployment and scaling
- Follows the existing project's web-first architecture

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

*No violations detected. All constitution principles are either satisfied or explicitly scoped out with clear rationale in the feature specification.*

## Phase 0: Research & Technology Decisions

**Completed**: ✅ `research.md` generated with all technology decisions

**Key Decisions Made**:

1. **Frontend Framework**: React via Docusaurus (existing)
   - Rationale: Native integration, no additional framework
   - Alternative: Separate SPA rejected (unnecessary complexity)

2. **HTTP Client**: Axios
   - Rationale: Better error handling, interceptors, timeout support
   - Alternative: Fetch API rejected (requires more manual error handling)

3. **State Management**: React Context + useReducer
   - Rationale: Sufficient for single-component chat state
   - Alternative: Redux rejected (overkill for this scope)

4. **UI Library**: Docusaurus/Infima CSS + Custom Styles
   - Rationale: Maintains visual consistency, no bundle bloat
   - Alternative: Material-UI rejected (200KB+ overhead)

5. **Testing**: React Testing Library + Vitest
   - Rationale: Best practices for React component testing
   - Alternative: Jest rejected (slower, less modern)

**All NEEDS CLARIFICATION items from Technical Context have been resolved in research.md**

## Phase 1: Design & Contracts

**Completed**: ✅ All Phase 1 artifacts generated

### 1. Data Model (`data-model.md`)

**Key Entities**:

- **ChatMessage**: User question or AI response with metadata
  - Fields: id, type, content, timestamp, status, sources, error
  - Validation: UUID format, character limits, status transitions
  - State flow: pending → sending → delivered/failed/timeout

- **ChatState**: Global chat interface state
  - Fields: messages[], isLoading, error, config
  - Managed via useReducer pattern
  - Actions: SEND_MESSAGE, RECEIVE_RESPONSE, MESSAGE_FAILED, etc.

- **SourceReference**: Document chunk citation
  - Fields: content, url, similarity_score, position
  - Validation: Score 0.0-1.0, valid URL, non-empty content

- **API Request/Response**: Backend communication contracts
  - AskRequest: { query, top_k?, request_id? }
  - AskResponse: { answer, sources, query, chunk_count, response_time_ms }
  - ErrorResponse: { error, message, status_code, type }

**Relationships**:
```
ChatState contains ChatMessage[]
ChatMessage (assistant) contains SourceReference[]
API flow: AskRequest → Backend → AskResponse → ChatMessage
```

### 2. API Contracts (`contracts/`)

**OpenAPI Specification** (`api-contract.yaml`):
- POST /ask: Submit question, receive answer with sources
- GET /health: Check API and service status
- Error responses: 400, 500, 503
- Request/response schemas with validation rules

**TypeScript Client Interface** (`frontend-api-client.ts`):
- `RagApiClient` interface with methods: ask(), checkHealth(), cancelRequest()
- `ApiResult<T>` type for consistent success/error handling
- Error codes, type guards, validation utilities
- Mock client for testing

### 3. Quickstart Guide (`quickstart.md`)

**Implementation Phases**:

**Phase 1**: Setup & Configuration
- Install dependencies (axios)
- Configure environment variables
- Create directory structure

**Phase 2**: Core Implementation
- Define TypeScript types
- Implement API client with retry logic
- Create ChatContext with useReducer
- Build main ChatBot component

**Phase 3**: UI Components
- MessageList with formatting
- InputBar with validation
- SourceList with collapsible accordion
- Styling with CSS modules

**Phase 4**: Docusaurus Integration
- Add ChatBot to Root.tsx
- Configure CORS on backend
- Test end-to-end flow

**Phase 5**: Testing
- Component tests with React Testing Library
- Manual testing checklist
- Integration tests with live backend

**Phase 6**: Deployment
- Production build
- Deploy backend with CORS
- Deploy frontend to GitHub Pages

### 4. Agent Context Update

**Completed**: ✅ Updated `CLAUDE.md` with:
- React + TypeScript specifics
- Docusaurus plugin architecture
- Frontend testing patterns
- API integration patterns

## Constitution Re-Check (Post-Design)

**Re-evaluated after Phase 1 design completion**:

### ✅ Modular Architecture (Re-verified)
- Component hierarchy clearly defined: ChatBot → ChatInterface → (MessageList, InputBar, SourceList)
- API client isolated in `src/lib/api/`
- State management centralized in ChatContext
- Each component has single responsibility

### ✅ Performance & Scalability (Re-verified)
- Lazy loading implemented: `React.lazy(() => import('./ChatInterface'))`
- Code splitting at component boundary
- Memoization strategy documented
- No performance anti-patterns identified

**Result**: All constitution principles remain satisfied after design phase

## Implementation Readiness

✅ **Research Complete**: All technology decisions documented
✅ **Data Models Defined**: TypeScript interfaces with validation
✅ **API Contracts Created**: OpenAPI spec + TypeScript client
✅ **Quickstart Guide Written**: Step-by-step implementation instructions
✅ **Agent Context Updated**: Technology stack documented for Claude Code

**Ready for `/sp.tasks`**: Yes

The planning phase is complete. All design artifacts are in place. The next step is to run `/sp.tasks` to generate the implementation task breakdown.

## Architecture Decisions Summary

### Decision 1: React Context for State Management
**Status**: ✅ Documented
**Rationale**: Lightweight, sufficient for chat state, no external dependencies
**Alternatives**: Redux (rejected: overkill), Zustand (rejected: unnecessary dependency)
**Impact**: Affects all component communication, but easily reversible

### Decision 2: Axios for HTTP Communication
**Status**: ✅ Documented
**Rationale**: Better error handling, interceptors, built-in timeout
**Alternatives**: Fetch (rejected: manual error handling), GraphQL (rejected: unnecessary for REST)
**Impact**: Minimal - HTTP client is abstracted behind interface

### Decision 3: Modal/Sidebar Chat UI
**Status**: ✅ Documented
**Rationale**: Non-intrusive, familiar pattern, works on all screen sizes
**Alternatives**: Embedded in page (rejected: reduces reading space), Full page (rejected: breaks flow)
**Impact**: Affects user experience but can be changed with CSS

**ADR Recommendation**:
📋 Architectural decision detected: **Frontend-Backend Integration Architecture** (React Context, Axios client, Modal UI pattern)

These decisions establish the foundation for the chat interface integration. While important, they are:
- **Localized Impact**: Primarily affect the chat component, not the entire application
- **Reversible**: Can be changed without major refactoring (interface abstraction helps)
- **Standard Patterns**: Using well-established React and HTTP client patterns

**Recommendation**: Document in implementation notes rather than formal ADR. If any decision proves problematic during implementation, revisit and create ADR at that time.

## Next Steps

1. **Run `/sp.tasks`** to generate detailed implementation tasks from this plan
2. **Begin Implementation** following quickstart.md guide
3. **Iterate** on design decisions if issues arise during implementation
4. **Test Thoroughly** using the testing strategy in quickstart.md
5. **Deploy** following the deployment guide in quickstart.md

## Branch Information

- **Feature Branch**: `003-frontend-backend-integration`
- **Base Branch**: `001-book-creation` (main)
- **Depends On**: Feature 002-rag-agent (backend must be deployed)

## Artifacts Generated

| Artifact | Path | Status | Purpose |
|----------|------|--------|---------|
| Research Report | `specs/003-frontend-backend-integration/research.md` | ✅ | Technology decisions |
| Data Model | `specs/003-frontend-backend-integration/data-model.md` | ✅ | Entity definitions |
| API Contract | `specs/003-frontend-backend-integration/contracts/api-contract.yaml` | ✅ | OpenAPI spec |
| Client Interface | `specs/003-frontend-backend-integration/contracts/frontend-api-client.ts` | ✅ | TypeScript client |
| Quickstart Guide | `specs/003-frontend-backend-integration/quickstart.md` | ✅ | Implementation steps |
| Implementation Plan | `specs/003-frontend-backend-integration/plan.md` | ✅ This file | Overall plan |

**Planning Phase Complete** ✅

---

*Generated by `/sp.plan` command on 2025-12-17*
