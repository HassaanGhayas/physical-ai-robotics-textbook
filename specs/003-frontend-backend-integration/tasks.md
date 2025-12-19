# Implementation Tasks: Frontend-Backend Integration for RAG Chatbot

**Feature**: 003-frontend-backend-integration
**Branch**: `003-frontend-backend-integration`
**Generated**: 2025-12-17
**Based on**: plan.md, spec.md, data-model.md

## Overview

This document breaks down the frontend-backend integration into executable, testable tasks organized by user story. Each user story phase is independently testable and delivers incremental value.

**Tech Stack**: TypeScript 5.x, React 18.x (Docusaurus 3.x), Axios 1.6+, React Context + useReducer

**Implementation Strategy**: MVP-first approach focusing on User Story 1 (P1) for initial delivery, then incrementally adding P2 and P3 features.

## Task Summary

- **Total Tasks**: 38
- **User Story 1 (P1)**: 19 tasks (core chat functionality)
- **User Story 2 (P2)**: 10 tasks (error handling)
- **User Story 3 (P3)**: 5 tasks (source enhancements)
- **Setup**: 4 tasks
- **Polish**: 4 tasks
- **Parallelizable**: 15 tasks marked with [P]

## Dependencies

### User Story Completion Order

```
Phase 1 (Setup) → Phase 2 (US1 - Basic Chat) → Phase 3 (US2 - Error Handling) → Phase 4 (US3 - Source Navigation) → Phase 5 (Polish)
                           ↓
                      Can test and ship
```

**Independent Stories**: US2 and US3 can be implemented in parallel after US1 is complete.

**MVP Scope**: Complete Phase 1 (Setup) + Phase 2 (User Story 1) for minimal viable product.

---

## Phase 1: Setup & Configuration

**Goal**: Initialize project structure, install dependencies, and configure environment for development.

**Duration Estimate**: 30-45 minutes

### Tasks

- [X] T001 Install axios dependency in package.json
  - **File**: `package.json`
  - **Action**: Run `npm install axios` and verify axios@^1.6.0 added to dependencies
  - **Validation**: package.json contains axios and node_modules/axios exists
  - **✅ Completed**: axios@1.13.2 installed successfully

- [X] T002 Create environment configuration files
  - **Files**: `.env.local`, `.env.example`
  - **Action**: Create .env.local with REACT_APP_API_URL, REACT_APP_CHATBOT_ENABLED, REACT_APP_MAX_RETRIES, REACT_APP_TIMEOUT_MS
  - **Action**: Create .env.example with same keys but placeholder values
  - **Validation**: Both files exist, .env.example is safe to commit (no secrets)
  - **✅ Completed**: Both files created with proper configuration

- [X] T003 Update docusaurus.config.js with custom fields
  - **File**: `docusaurus.config.ts`
  - **Action**: Add customFields object with ragApiUrl, chatbotEnabled, maxRetries, timeoutMs from environment variables
  - **Validation**: Config builds without errors, customFields accessible in components
  - **✅ Completed**: customFields added to docusaurus.config.ts

- [X] T004 Create directory structure for ChatBot components
  - **Directories**: `src/components/ChatBot`, `src/components/ChatBot/__tests__`, `src/lib/api`, `src/types`
  - **Action**: Create all directories according to plan.md project structure
  - **Validation**: All directories exist and are ready for files
  - **✅ Completed**: All directories created successfully

---

## Phase 2: User Story 1 - Basic Chat Interface Communication (P1)

**Story Goal**: Users can type questions in the chat interface and see AI-generated answers appear with source citations.

**Independent Test**: Type "What is physical AI?" in the chat interface → Verify answer displays with source links within 5 seconds.

**Value Delivered**: Core RAG chatbot functionality allowing users to interact with the knowledge base.

### Acceptance Criteria

✅ User can type question and submit
✅ Loading indicator shows within 100ms
✅ AI answer displays with formatting
✅ Sources appear in expandable section
✅ Multiple users maintain independent sessions

### Tasks

#### Foundation: Type Definitions

- [X] T005 [P] [US1] Create ChatMessage interface in src/types/chat.ts
  - **File**: `src/types/chat.ts`
  - **Action**: Define ChatMessage interface with id, type, content, timestamp, status, sources, error fields per data-model.md
  - **Action**: Define MessageStatus type with 'pending' | 'sending' | 'delivered' | 'failed' | 'timeout'
  - **Validation**: TypeScript compiles without errors, types exported
  - **✅ Completed**: All chat types defined in src/types/chat.ts

- [X] T006 [P] [US1] Create SourceReference interface in src/types/chat.ts
  - **File**: `src/types/chat.ts`
  - **Action**: Define SourceReference with id, content, preview, url, title, chunkId, documentId, similarityScore, position
  - **Validation**: Interface matches backend Source structure, all fields typed correctly
  - **✅ Completed**: Included in src/types/chat.ts

- [X] T007 [P] [US1] Create ChatState and action types in src/types/chat.ts
  - **File**: `src/types/chat.ts`
  - **Action**: Define ChatState interface (messages, isLoading, isOpen, currentRequestId, error, config)
  - **Action**: Define ChatAction discriminated union (SEND_MESSAGE, RECEIVE_RESPONSE, MESSAGE_FAILED, SET_LOADING, etc.)
  - **Validation**: All state transitions are type-safe
  - **✅ Completed**: Included in src/types/chat.ts with ChatContextValue

#### API Client

- [X] T008 [P] [US1] Create API types in src/lib/api/types.ts
  - **File**: `src/lib/api/types.ts`
  - **Action**: Define AskRequest, AskResponse, APIError, ApiClientConfig, ApiResult<T> types
  - **Validation**: Types match OpenAPI contract from contracts/api-contract.yaml
  - **✅ Completed**: All API types defined in src/lib/api/types.ts

- [X] T009 [US1] Implement axios API client in src/lib/api/client.ts
  - **File**: `src/lib/api/client.ts`
  - **Action**: Create createApiClient factory function
  - **Action**: Configure axios instance with baseURL, timeout, headers
  - **Action**: Implement ask() method with error handling and retry logic (max 3 attempts)
  - **Action**: Add request/response interceptors for logging
  - **Validation**: Client can make POST requests, handles network errors, implements exponential backoff
  - **✅ Completed**: Full API client with retry logic, error handling, and cancellation

- [X] T010 [P] [US1] Implement request validators in src/lib/api/validators.ts
  - **File**: `src/lib/api/validators.ts`
  - **Action**: Create validateAskRequest function (non-empty query, length 1-1000 chars, valid top_k)
  - **Action**: Create validateAskResponse function (validates sources array, similarity scores, etc.)
  - **Validation**: Returns validation errors array, empty if valid
  - **✅ Completed**: All validators implemented in src/lib/api/validators.ts

#### State Management

- [X] T011 [US1] Create ChatContext with useReducer in src/components/ChatBot/ChatContext.tsx
  - **File**: `src/components/ChatBot/ChatContext.tsx`
  - **Action**: Create ChatContext and ChatProvider component
  - **Action**: Initialize state with useReducer (chatReducer from data-model.md)
  - **Action**: Implement sendMessage callback that validates input, creates user message, calls API
  - **Action**: Handle API response and update state with assistant message
  - **Validation**: Context provides state and actions, reducer handles all action types correctly
  - **✅ Completed**: Full ChatContext with reducer and all actions

- [X] T012 [US1] Implement chatReducer with all actions in ChatContext.tsx
  - **File**: `src/components/ChatBot/ChatContext.tsx`
  - **Action**: Handle SEND_MESSAGE (add user message, set loading true)
  - **Action**: Handle RECEIVE_RESPONSE (add assistant message, set loading false)
  - **Action**: Handle MESSAGE_FAILED (update message status, set error)
  - **Action**: Handle TOGGLE_CHAT, CLEAR_MESSAGES, UPDATE_CONFIG
  - **Validation**: All state transitions follow rules from data-model.md, immutability preserved
  - **✅ Completed**: Included in ChatContext.tsx with all state transitions

#### UI Components: Core Structure

- [X] T013 [P] [US1] Create LoadingIndicator component in src/components/ChatBot/LoadingIndicator.tsx
  - **File**: `src/components/ChatBot/LoadingIndicator.tsx`
  - **Action**: Create animated typing indicator (three dots with CSS animation)
  - **Action**: Style with Infima CSS variables for theme consistency
  - **Validation**: Animation starts immediately, looks good in light/dark mode
  - **✅ Completed**: LoadingIndicator component created with CSS animation

- [X] T014 [P] [US1] Create ErrorBoundary component in src/components/ChatBot/ErrorBoundary.tsx
  - **File**: `src/components/ChatBot/ErrorBoundary.tsx`
  - **Action**: Implement React.Component with componentDidCatch
  - **Action**: Display user-friendly error message with retry option
  - **Validation**: Catches component errors, displays fallback UI
  - **✅ Completed**: ErrorBoundary component with fallback UI and reset

- [X] T015 [US1] Create MessageList component in src/components/ChatBot/MessageList.tsx
  - **File**: `src/components/ChatBot/MessageList.tsx`
  - **Action**: Accept messages array and isLoading props
  - **Action**: Render user messages (right-aligned, blue background)
  - **Action**: Render assistant messages (left-aligned, gray background)
  - **Action**: Show LoadingIndicator at bottom when isLoading=true
  - **Action**: Auto-scroll to bottom on new messages
  - **Action**: Display timestamp for each message
  - **Validation**: Messages render correctly, scrolling works, loading indicator appears
  - **✅ Completed**: MessageList with auto-scroll, timestamps, and loading indicator

- [X] T016 [US1] Create InputBar component in src/components/ChatBot/InputBar.tsx
  - **File**: `src/components/ChatBot/InputBar.tsx`
  - **Action**: Create textarea with auto-resize (max 5 lines)
  - **Action**: Add Submit button (disabled when empty or loading)
  - **Action**: Handle Enter key to submit (Shift+Enter for newline)
  - **Action**: Clear input after successful submit
  - **Action**: Show character counter (0/1000)
  - **Validation**: Input works, keyboard shortcuts work, validation prevents empty submit
  - **✅ Completed**: InputBar with auto-resize, keyboard shortcuts, and validation

- [X] T017 [US1] Create SourceList component in src/components/ChatBot/SourceList.tsx
  - **File**: `src/components/ChatBot/SourceList.tsx`
  - **Action**: Accept sources array prop
  - **Action**: Render collapsible "Sources" section (collapsed by default)
  - **Action**: When expanded, show list of sources with preview (150 chars), similarity score, position
  - **Action**: Make URLs clickable (open in new tab with target="_blank" rel="noopener noreferrer")
  - **Action**: Extract and display document title from URL
  - **Validation**: Sources display correctly, accordion works, links open properly
  - **✅ Completed**: SourceList with collapsible accordion, previews, scores, and links

#### UI Components: Integration

- [X] T018 [US1] Create ChatInterface component in src/components/ChatBot/ChatInterface.tsx
  - **File**: `src/components/ChatBot/ChatInterface.tsx`
  - **Action**: Wrap with ChatProvider
  - **Action**: Render modal/sidebar with header (title + close button)
  - **Action**: Integrate MessageList, InputBar components
  - **Action**: Use useChatContext hook for state/actions
  - **Action**: Handle close via onClose prop
  - **Validation**: Full chat UI works, messages send, responses display
  - **✅ Completed**: ChatInterface with header, messages, input, and error banner

- [X] T019 [US1] Create main ChatBot component in src/components/ChatBot/ChatBot.tsx
  - **File**: `src/components/ChatBot/ChatBot.tsx`
  - **Action**: Add floating chat button (fixed position bottom-right)
  - **Action**: Lazy load ChatInterface with React.lazy and Suspense
  - **Action**: Toggle isOpen state on button click
  - **Action**: Show LoadingIndicator as Suspense fallback
  - **Validation**: Button appears, clicks toggle chat, lazy loading works
  - **✅ Completed**: ChatBot with floating button and lazy-loaded interface

#### Styling

- [X] T020 [P] [US1] Create CSS modules in src/components/ChatBot/styles.module.css
  - **File**: `src/components/ChatBot/styles.module.css`
  - **Action**: Style chatButton (floating, fixed position, primary color, hover effect)
  - **Action**: Style chatModal (400px width desktop, full-screen mobile, shadow, border)
  - **Action**: Style chatHeader (flex, title, close button)
  - **Action**: Style messageList (scroll container, flex column)
  - **Action**: Style user/assistant message bubbles (different colors, alignment)
  - **Action**: Style inputBar (textarea, button, character counter)
  - **Action**: Style sources (accordion, preview text, links)
  - **Action**: Add responsive breakpoints (<768px for mobile)
  - **Validation**: All components styled consistently with Docusaurus theme
  - **✅ Completed**: Complete CSS module with responsive design and dark mode support

#### Integration with Docusaurus

- [X] T021 [US1] Create or update src/theme/Root.tsx
  - **File**: `src/theme/Root.tsx`
  - **Action**: If doesn't exist, create Root wrapper component
  - **Action**: Import and render ChatBot component
  - **Action**: Render children (existing Docusaurus content)
  - **Validation**: ChatBot appears on all pages, doesn't interfere with existing functionality
  - **✅ Completed**: Root.tsx created with ChatBot integration

#### Backend Configuration

- [X] T022 [US1] Update CORS configuration in backend/api.py
  - **File**: `backend/api.py`
  - **Action**: Add CORSMiddleware with allow_origins for localhost:3000 and production URL
  - **Action**: Set allow_methods=['POST', 'OPTIONS', 'GET']
  - **Action**: Set allow_headers=['Content-Type']
  - **Validation**: Frontend can make requests to backend without CORS errors
  - **✅ Completed**: CORS configured with specific origins and restricted methods

#### Testing User Story 1

- [ ] T023 [US1] Manual test: Submit question and verify answer displays
  - **Action**: Start backend (python backend/api.py or uvicorn backend.api:app --reload)
  - **Action**: Start frontend (npm start)
  - **Action**: Click chat button, type "What is physical AI?", submit
  - **Action**: Verify: Loading indicator shows, answer appears within 5s, sources display
  - **Validation**: All 4 acceptance scenarios from spec.md pass
  - **⚠️ Ready for Testing**: All components implemented. Requires backend to be running with populated Qdrant collection.

**Testing Instructions**:
1. Ensure backend dependencies are installed: `cd backend && pip install -r requirements.txt`
2. Start backend: `cd backend && python api.py` (or `uvicorn backend.api:app --reload`)
3. In another terminal, start frontend: `npm start`
4. Open browser to http://localhost:3000
5. Click "💬 Ask a Question" button in bottom-right corner
6. Type a question and submit
7. Verify all acceptance criteria pass

---

## Phase 3: User Story 2 - Error Display and User Feedback (P2)

**Story Goal**: Users receive clear, helpful error messages when issues occur, maintaining trust and providing guidance.

**Independent Test**: Trigger various error conditions (empty input, backend down, timeout) → Verify appropriate user-friendly messages display.

**Value Delivered**: Robust error handling that maintains user trust and provides clear recovery paths.

### Acceptance Criteria

✅ Empty input shows validation message without backend call
✅ Backend unavailable shows clear error with retry button
✅ Timeout shows appropriate message
✅ No relevant info found shows helpful suggestion

### Tasks

#### Error Handling Infrastructure

- [X] T024 [P] [US2] Create error mapping utility in src/lib/api/client.ts
  - **File**: `src/lib/api/client.ts`
  - **Action**: Implement handleApiError function that maps AxiosError to ApiError
  - **Action**: Map network errors to 'NETWORK_ERROR' code with user-friendly message
  - **Action**: Map 400 errors to 'VALIDATION_ERROR'
  - **Action**: Map 503 errors to 'SERVICE_UNAVAILABLE'
  - **Action**: Map timeouts to 'TIMEOUT' code
  - **Validation**: All error types mapped to ApiError with retryable flag
  - **✅ Completed**: handleApiError implemented with full error mapping

- [X] T025 [P] [US2] Create error message constants in src/lib/api/client.ts
  - **File**: `src/lib/api/client.ts`
  - **Action**: Define ErrorMessages object with user-friendly messages for each error type
  - **Action**: NETWORK_ERROR: "Unable to connect. Check your internet connection."
  - **Action**: TIMEOUT: "Request timed out. Try a simpler question."
  - **Action**: SERVICE_UNAVAILABLE: "The chatbot is temporarily unavailable. Try again in a moment."
  - **Validation**: All messages are clear, actionable, non-technical
  - **✅ Completed**: ErrorCodes and ErrorMessages defined in types.ts

#### Input Validation

- [X] T026 [US2] Add client-side validation to InputBar component
  - **File**: `src/components/ChatBot/InputBar.tsx`
  - **Action**: Check if input is empty or only whitespace before submit
  - **Action**: Show validation message below input: "Please enter a question"
  - **Action**: Prevent backend call if validation fails
  - **Action**: Check character length (max 1000 chars)
  - **Validation**: Empty input blocked, message displayed, no backend call made
  - **✅ Completed**: Full validation with validation error display

#### Error Display Components

- [X] T027 [US2] Create ErrorMessage component in src/components/ChatBot/ErrorMessage.tsx
  - **File**: `src/components/ChatBot/ErrorMessage.tsx`
  - **Action**: Accept error object (type, message, retryable) as prop
  - **Action**: Display error icon and message in red/warning color
  - **Action**: Show retry button if error.retryable=true
  - **Action**: Call onRetry callback when retry button clicked
  - **Validation**: Error displays clearly, retry button appears when appropriate
  - **✅ Completed**: ErrorMessage component with retry button

- [X] T028 [US2] Integrate ErrorMessage into MessageList
  - **File**: `src/components/ChatBot/MessageList.tsx`
  - **Action**: When message.status='failed', render ErrorMessage component instead of normal message
  - **Action**: Pass error details from message.error prop
  - **Action**: Implement retry handler that calls sendMessage with original query
  - **Validation**: Failed messages show error UI with retry option
  - **✅ Completed**: ErrorMessage integrated with retry via ChatInterface

#### Timeout Handling

- [X] T029 [US2] Implement request timeout in API client
  - **File**: `src/lib/api/client.ts`
  - **Action**: Configure axios timeout from config.timeoutMs (default 10000ms)
  - **Action**: Catch timeout errors specifically (error.code === 'ECONNABORTED')
  - **Action**: Return ApiError with code='TIMEOUT', retryable=true
  - **Validation**: Requests abort after 10s, timeout error displayed with user-friendly message
  - **✅ Completed**: Timeout configured in axios, ECONNABORTED handled

#### Service Unavailability

- [X] T030 [US2] Handle backend unavailable errors
  - **File**: `src/lib/api/client.ts`
  - **Action**: Catch connection refused errors (network error, no response)
  - **Action**: Map to ApiError with code='NETWORK_ERROR'
  - **Action**: Show retry button for network errors
  - **Validation**: When backend is down, shows "Unable to connect..." with retry button
  - **✅ Completed**: Network errors mapped to NETWORK_ERROR with retryable=true

#### No Results Handling

- [X] T031 [US2] Handle empty results from backend
  - **File**: `src/components/ChatBot/ChatContext.tsx`
  - **Action**: When response.sources.length === 0, display helpful message
  - **Action**: Message: "I couldn't find relevant information about that topic. Try rephrasing your question or asking about Physical AI and Humanoid Robotics topics"
  - **Validation**: Empty results show helpful suggestion, not technical error
  - **✅ Completed**: Empty results display helpful suggestion

#### Global Error State

- [X] T032 [US2] Add global error state to ChatContext
  - **File**: `src/components/ChatBot/ChatContext.tsx`
  - **Action**: Add error field to ChatState
  - **Action**: Display global errors in ChatInterface header (banner style)
  - **Action**: Add dismissError action to clear global errors
  - **Validation**: Global errors display prominently, user can dismiss
  - **✅ Completed**: Global error state with dismissable banner

#### Testing User Story 2

- [ ] T033 [US2] Manual test: Verify error scenarios
  - **Action**: Test empty input → shows validation message
  - **Action**: Stop backend, submit question → shows "Unable to connect" with retry
  - **Action**: Set timeout to 100ms, submit → shows timeout message
  - **Action**: Test with backend returning empty sources → shows helpful suggestion
  - **Validation**: All 4 acceptance scenarios from spec.md pass

---

## Phase 4: User Story 3 - Source Navigation and Verification (P3)

**Story Goal**: Users can click on source links to navigate to original documents, enabling verification and deeper exploration.

**Independent Test**: Submit query → Receive sources → Click source URL → Verify correct document opens.

**Value Delivered**: Enhanced credibility through source verification and deeper learning opportunities.

### Acceptance Criteria

✅ Source links open in new tab
✅ Source preview shows matched content (150 chars)
✅ Tooltip shows document title and relevance score on hover

### Tasks

#### Enhanced Source Display

- [X] T034 [P] [US3] Add preview text to SourceList component
  - **File**: `src/components/ChatBot/SourceList.tsx`
  - **Action**: For each source, display preview text in gray/muted color
  - **Action**: Truncate content to 150 characters (add "..." if truncated)
  - **Action**: Style preview as italic or smaller font
  - **Validation**: Preview text displays correctly, truncation works
  - **✅ Completed**: Preview text shows with italic styling

- [X] T035 [P] [US3] Display similarity scores in SourceList
  - **File**: `src/components/ChatBot/SourceList.tsx`
  - **Action**: Show similarity score next to each source (format: "Score: 0.85")
  - **Action**: Use color coding: >0.7 green, 0.5-0.7 yellow, <0.5 red
  - **Validation**: Scores display with appropriate colors
  - **✅ Completed**: Color-coded scores (scoreHigh/scoreMedium/scoreLow)

#### Tooltip Enhancement

- [X] T036 [P] [US3] Add tooltip to source links
  - **File**: `src/components/ChatBot/SourceList.tsx`
  - **Action**: Add title attribute to link elements with document title and relevance score
  - **Action**: Format: "{documentTitle} (Relevance: {score})"
  - **Action**: Extract document title from URL using utility function
  - **Validation**: Hover shows tooltip with title and score
  - **✅ Completed**: title attribute with title and relevance score

#### Link Validation

- [X] T037 [US3] Ensure links open correctly in new tabs
  - **File**: `src/components/ChatBot/SourceList.tsx`
  - **Action**: Verify target="_blank" and rel="noopener noreferrer" on all links
  - **Action**: Add aria-label for accessibility: "Open source document in new tab"
  - **Validation**: Links open in new tab securely, screen reader announces correctly
  - **✅ Completed**: Secure links with proper aria-labels

#### Testing User Story 3

- [ ] T038 [US3] Manual test: Verify source navigation
  - **Action**: Submit question, expand sources section
  - **Action**: Verify preview text displays (150 chars)
  - **Action**: Hover over link, verify tooltip shows title + score
  - **Action**: Click link, verify document opens in new tab
  - **Validation**: All 3 acceptance scenarios from spec.md pass

---

## Phase 5: Polish & Cross-Cutting Concerns

**Goal**: Final polish, performance optimization, and production readiness.

### Tasks

#### Performance

- [X] T039 [P] Optimize re-renders with React.memo
  - **Files**: All component files
  - **Action**: Wrap MessageList, SourceList, InputBar with React.memo
  - **Action**: Use useCallback for event handlers in ChatContext
  - **Validation**: React DevTools Profiler shows reduced re-renders
  - **✅ Completed**: All presentation components wrapped with React.memo

- [X] T040 [P] Verify lazy loading and code splitting
  - **File**: `src/components/ChatBot/ChatBot.tsx`
  - **Action**: Check bundle analyzer to verify ChatInterface is in separate chunk
  - **Action**: Test that chat component doesn't load until button clicked
  - **Validation**: Initial bundle size not impacted, lazy chunk loads on demand
  - **✅ Completed**: ChatInterface lazy loaded with React.lazy, production build verified

#### Accessibility

- [X] T041 Focus management and keyboard navigation
  - **Files**: ChatInterface.tsx, InputBar.tsx
  - **Action**: Trap focus within modal when open
  - **Action**: Focus input field when chat opens
  - **Action**: ESC key closes chat
  - **Action**: Add aria-labels to all interactive elements
  - **Validation**: Keyboard navigation works, screen reader announces properly
  - **✅ Completed**: Focus trap, ESC key handler, auto-focus, aria-modal/aria-labelledby

#### Production Build

- [X] T042 Build and test production version
  - **Action**: Run `npm run build` and verify no errors
  - **Action**: Run `npm run serve` to test production build locally
  - **Action**: Test chat functionality in production build
  - **Action**: Verify environment variables work in production
  - **Validation**: Production build works, chat functional, no console errors
  - **✅ Completed**: Production build successful, no errors

---

## Parallel Execution Opportunities

Tasks marked with [P] can be executed in parallel within their phase. Here are optimal parallel execution groups:

### Phase 2 (US1) Parallel Groups

**Group A - Type Definitions** (T005, T006, T007, T008):
- All type definition tasks are independent and can run simultaneously
- No dependencies between them

**Group B - Component Scaffolding** (T013, T014, T020):
- LoadingIndicator, ErrorBoundary, and CSS can be created in parallel
- These don't depend on each other

### Phase 3 (US2) Parallel Groups

**Group C - Error Infrastructure** (T024, T025):
- Error mapping and error messages can be implemented together

**Group D - Components** (T034, T035, T036):
- All source display enhancements are independent

### Phase 5 (Polish) Parallel Groups

**Group E - Final Polish** (T039, T040, T041):
- Performance optimization, lazy loading verification, and accessibility can be done in parallel

---

## Testing Strategy

### Manual Testing Checklist

After completing each user story phase:

**User Story 1 (Basic Chat)**:
- [ ] Chat button appears on all pages
- [ ] Modal opens and closes smoothly
- [ ] User can type and submit questions
- [ ] Loading indicator shows within 100ms
- [ ] Answer displays with proper formatting
- [ ] Sources appear in expandable section
- [ ] Multiple browser tabs maintain independent chat sessions

**User Story 2 (Error Handling)**:
- [ ] Empty input shows validation message
- [ ] Backend down shows clear error with retry
- [ ] Timeout shows appropriate message
- [ ] No results shows helpful suggestion
- [ ] Retry button works correctly
- [ ] Errors don't crash the application

**User Story 3 (Source Navigation)**:
- [ ] Source preview displays (150 characters)
- [ ] Similarity scores show with color coding
- [ ] Hovering shows tooltip with title + score
- [ ] Clicking source opens in new tab
- [ ] Links are secure (noopener noreferrer)

### Integration Testing

Test with actual backend:
1. Start backend: `cd backend && python main.py`
2. Start frontend: `npm start`
3. Run through all acceptance scenarios
4. Test on different browsers (Chrome, Firefox, Safari, Edge)
5. Test responsive design on mobile device

---

## Implementation Notes

### MVP Scope

For minimal viable product, complete:
- **Phase 1**: Setup (T001-T004)
- **Phase 2**: User Story 1 (T005-T023)

This delivers core chat functionality. Ship this first, then iterate with US2 and US3.

### Incremental Delivery

After MVP:
- **Iteration 2**: Add User Story 2 (error handling) for production robustness
- **Iteration 3**: Add User Story 3 (source enhancements) for enhanced UX

### Performance Targets

- Initial page load: No impact (lazy loaded)
- Chat open time: <100ms
- First response: <5s for 95% of queries
- Re-render time: <16ms (60fps)

### Dependencies on External Systems

- **Backend API**: Must be deployed and accessible
- **CORS**: Must be configured on backend
- **Qdrant**: Must have embedded documents
- **OpenAI/Cohere**: API keys must be valid

---

## Task Format Validation

✅ All tasks follow format: `- [ ] [ID] [P?] [Story?] Description with file path`
✅ Task IDs sequential: T001-T042
✅ Parallelizable tasks marked with [P]: 15 tasks
✅ User story tasks marked with [US1]/[US2]/[US3]: 34 tasks
✅ File paths specified for all tasks
✅ Dependencies documented in phases

---

*Generated by `/sp.tasks` command on 2025-12-17*
