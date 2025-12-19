# Research Report: Frontend-Backend Integration for RAG Chatbot

**Feature**: 003-frontend-backend-integration
**Date**: 2025-12-17
**Phase**: Phase 0 - Research & Technology Decisions

## Overview

This research document resolves all technical unknowns identified in the implementation plan's Technical Context section. The goal is to establish the technical foundation for integrating the Docusaurus frontend with the FastAPI RAG Agent backend.

## Technology Decisions

### 1. Frontend Framework & Integration Pattern

**Decision**: React-based Docusaurus plugin with custom chat component

**Rationale**:
- Docusaurus is already built on React, providing native integration
- Custom React components can be added as swizzled components or plugins
- Maintains consistency with existing Docusaurus UI patterns
- No additional framework overhead

**Alternatives Considered**:
- Web Components: More isolated but requires additional build setup
- iFrame embed: Simple but limited styling and communication
- Separate SPA: Adds complexity with cross-origin and routing

**Implementation Approach**:
- Create a custom React component for chat interface
- Integrate as a Docusaurus theme component or plugin
- Use React hooks for state management
- Position as modal or sidebar overlay

### 2. HTTP Communication Layer

**Decision**: Fetch API with axios for enhanced features

**Rationale**:
- Fetch API is native and widely supported
- Axios provides better error handling, request cancellation, and interceptors
- TypeScript support for request/response typing
- Automatic JSON transformation
- Built-in timeout configuration

**Alternatives Considered**:
- Native Fetch only: Simpler but requires manual error handling
- XMLHttpRequest: Outdated, callback-based
- GraphQL client: Overkill for simple REST API

**Configuration**:
```typescript
// API client configuration
baseURL: process.env.REACT_APP_API_URL || 'http://localhost:8000'
timeout: 10000 // 10 seconds per requirement
headers: {
  'Content-Type': 'application/json'
}
```

### 3. State Management Strategy

**Decision**: React Context API + useReducer hook

**Rationale**:
- Lightweight solution for chat state (messages, loading, errors)
- No external dependencies (Redux, MobX)
- Sufficient for single-component state management
- Easy to understand and maintain
- Follows React best practices

**State Structure**:
```typescript
interface ChatState {
  messages: ChatMessage[];
  isLoading: boolean;
  error: ErrorState | null;
  requestId: string | null;
}
```

**Alternatives Considered**:
- Redux: Overkill for simple chat state
- Zustand: Adds dependency for minimal benefit
- Component state only: Insufficient for complex interactions

### 4. Error Handling Pattern

**Decision**: Centralized error boundary + user-friendly messages

**Rationale**:
- React Error Boundaries catch component-level errors
- API errors mapped to user-friendly messages
- Retry logic for transient failures
- Clear error states in UI

**Error Categories**:
1. **Validation Errors** (400): "Please enter a valid question"
2. **Network Errors**: "Unable to connect. Check your internet connection."
3. **Service Unavailable** (503): "The chatbot is temporarily unavailable. Try again in a moment."
4. **Timeout Errors**: "Request timed out. Try a simpler question."
5. **Unknown Errors** (500): "Something went wrong. Please try again."

### 5. UI Component Library

**Decision**: Use existing Docusaurus/Infima CSS + custom styles

**Rationale**:
- Maintains visual consistency with documentation site
- No additional bundle size from external UI library
- Infima provides base styling and theme variables
- Custom CSS modules for chat-specific components

**Alternatives Considered**:
- Material-UI: Heavy bundle, different design language
- Chakra UI: Adds 200KB+ to bundle
- Tailwind CSS: Requires build configuration changes

### 6. Loading & Feedback Indicators

**Decision**: Animated typing indicator + skeleton placeholders

**Rationale**:
- Provides immediate feedback (within 100ms requirement)
- Typing animation indicates AI is "thinking"
- Skeleton loaders for predictable content layout
- Uses CSS animations (no JavaScript libraries needed)

**Implementation**:
- Display typing indicator immediately on submit
- Show skeleton for answer and sources sections
- Animate message appearance for smooth UX

### 7. Source Citation Rendering

**Decision**: Collapsible accordion with inline previews

**Rationale**:
- Keeps answer text prominent
- Sources available but not overwhelming
- Preview text (150 chars) provides context
- Similarity scores show relevance
- Clickable links open in new tab

**Format**:
```
Answer: [AI-generated response]

Sources ▼
  [1] Document Title (Score: 0.85)
      "Preview text showing matched content..."
      → View full document
  [2] Another Document (Score: 0.78)
      "More preview text from the source..."
      → View full document
```

### 8. Responsive Design Strategy

**Decision**: Mobile-first CSS with breakpoints

**Rationale**:
- Docusaurus already responsive
- Chat component must work on all screen sizes
- Modal on desktop, full-screen on mobile
- Touch-friendly tap targets (min 44x44px)

**Breakpoints**:
- Mobile: < 768px (full-screen overlay)
- Tablet: 768px - 996px (right sidebar)
- Desktop: > 996px (modal dialog)

### 9. Testing Strategy

**Decision**: React Testing Library + Vitest

**Rationale**:
- React Testing Library for component tests
- Vitest for fast, ESM-native testing
- MSW (Mock Service Worker) for API mocking
- Tests user interactions, not implementation details

**Test Coverage**:
1. User can type and submit questions
2. Loading states display correctly
3. Answers render with proper formatting
4. Sources are clickable and expandable
5. Error messages display appropriately
6. Retry functionality works

### 10. Performance Optimization

**Decision**: Code splitting + lazy loading

**Rationale**:
- Chat component loaded only when triggered
- Reduces initial bundle size
- React.lazy() for component-level splitting
- Suspense boundary for loading state

**Implementation**:
```typescript
const ChatInterface = React.lazy(() => import('./ChatInterface'));

// In parent component
<React.Suspense fallback={<LoadingSpinner />}>
  {showChat && <ChatInterface />}
</React.Suspense>
```

## Backend API Contract Understanding

**Architecture**: The backend uses **Cohere** for both embeddings and answer generation, with **Qdrant** for vector storage. This aligns with the project constitution's emphasis on "Cohere Models for RAG".

**RAG Pipeline**:
1. Query → Cohere embeddings
2. Vector search in Qdrant
3. Retrieved documents → Cohere chat model (command-r)
4. Answer + sources → Frontend

### Endpoint: POST /ask

**Request Format**:
```json
{
  "query": "What is physical AI?",
  "top_k": 5  // Optional, defaults to 5
}
```

**Response Format** (Success):
```json
{
  "answer": "Physical AI refers to...",
  "sources": [
    {
      "content": "Matched text chunk...",
      "url": "https://example.com/doc",
      "chunk_id": "chunk_123",
      "document_id": "doc_456",
      "similarity_score": 0.85,
      "position": 1
    }
  ],
  "query": "What is physical AI?",
  "chunk_count": 5,
  "response_time_ms": 3200
}
```

**Response Format** (Error):
```json
{
  "error": true,
  "message": "Query cannot be empty",
  "status_code": 400,
  "type": "ValidationError"
}
```

### CORS Configuration

**Required Headers** (Backend):
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["https://hassaanghayas.github.io", "http://localhost:3000"],
    allow_methods=["POST", "OPTIONS"],
    allow_headers=["Content-Type"],
    allow_credentials=False,
    max_age=600
)
```

## Technology Stack Summary

| Layer | Technology | Version | Purpose |
|-------|-----------|---------|---------|
| **Frontend Framework** | React (via Docusaurus) | 18.x | UI components |
| **HTTP Client** | Axios | 1.6+ | API communication |
| **State Management** | React Context + useReducer | Built-in | Chat state |
| **Styling** | Infima + CSS Modules | Built-in | Component styling |
| **Testing** | React Testing Library + Vitest | Latest | Component testing |
| **Type Safety** | TypeScript | 5.x | Type checking |
| **Backend** | FastAPI (existing) | 0.104+ | REST API |
| **AI Model** | Cohere (command-r) | Latest | Embeddings & generation |
| **Vector DB** | Qdrant Cloud | Latest | Document storage |
| **Deployment** | GitHub Pages (existing) | N/A | Static hosting |

## Best Practices & Patterns

### 1. Error Handling
- Always provide user-friendly error messages
- Implement retry logic with exponential backoff (3 attempts max)
- Log errors to console for debugging (production: use error tracking service)
- Show retry button for failed requests

### 2. Loading States
- Display loading indicator within 100ms of submit
- Prevent duplicate submissions while loading
- Show progress for long-running requests
- Allow request cancellation if user navigates away

### 3. Accessibility
- Keyboard navigation support (Enter to submit, Escape to close)
- ARIA labels for screen readers
- Focus management (trap focus in modal)
- Color contrast ratios meet WCAG AA standards
- Skip link to bypass chat if needed

### 4. Performance
- Debounce input validation (300ms)
- Lazy load chat component
- Optimize re-renders with React.memo
- Use CSS animations over JavaScript
- Compress API payloads (gzip)

### 5. Security
- Sanitize user input before display
- Use CSP headers to prevent XSS
- Validate all backend responses
- No sensitive data in client-side code
- HTTPS for all API calls

## Integration Points

### 1. Docusaurus Configuration

**File**: `docusaurus.config.js`

Add custom fields for API configuration:
```javascript
module.exports = {
  // ... existing config
  customFields: {
    ragApiUrl: process.env.RAG_API_URL || 'http://localhost:8000',
    chatbotEnabled: true,
  },
};
```

### 2. Component Location

**Recommended Structure**:
```
src/
  components/
    ChatBot/
      ChatBot.tsx          # Main container
      ChatInterface.tsx    # Chat UI
      MessageList.tsx      # Message rendering
      InputBar.tsx         # Query input
      SourceList.tsx       # Source citations
      ErrorBoundary.tsx    # Error handling
      types.ts             # TypeScript types
      api.ts               # API client
      styles.module.css    # Component styles
      ChatBot.test.tsx     # Tests
```

### 3. Swizzling Strategy

**Option A**: Swizzle NavbarItem (Recommended)
- Add chat button to navbar
- Consistent with Docusaurus patterns
- Easy to discover

**Option B**: Swizzle Root wrapper
- Add floating chat button
- Always visible
- More flexible positioning

### 4. Environment Variables

**File**: `.env` (not committed)
```bash
REACT_APP_API_URL=https://api.example.com
REACT_APP_CHATBOT_ENABLED=true
```

**File**: `.env.example` (committed)
```bash
REACT_APP_API_URL=http://localhost:8000
REACT_APP_CHATBOT_ENABLED=true
```

## Risks & Mitigation

### Risk 1: Backend Not Deployed
**Mitigation**:
- Check for deployed backend URL in config
- Show "Coming Soon" message if not available
- Provide graceful degradation

### Risk 2: CORS Issues
**Mitigation**:
- Configure CORS on backend before frontend integration
- Test with actual deployed URLs
- Document CORS requirements

### Risk 3: Rate Limiting
**Mitigation**:
- Implement client-side rate limiting
- Show warning if user submits too quickly
- Queue requests if needed

### Risk 4: Large Response Payload
**Mitigation**:
- Limit source chunks to 5 by default
- Paginate sources if more than 10
- Compress response with gzip

## Next Steps (Phase 1)

1. Create data model definitions for chat entities
2. Design API client interface
3. Define component props and state shapes
4. Create API contract documentation
5. Update agent context with React/TypeScript specifics
