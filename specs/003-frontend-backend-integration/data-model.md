# Data Model: Frontend-Backend Integration

**Feature**: 003-frontend-backend-integration
**Date**: 2025-12-17
**Phase**: Phase 1 - Design & Contracts

## Overview

This document defines all data entities, their fields, validation rules, and state transitions for the frontend-backend integration. These models ensure type safety and consistent data handling across the chat interface.

## Frontend Data Models

### 1. ChatMessage

Represents a single message in the conversation (user question or AI response).

**TypeScript Interface**:
```typescript
interface ChatMessage {
  id: string;                    // Unique message identifier (UUID)
  type: 'user' | 'assistant';    // Message sender type
  content: string;               // Message text content
  timestamp: Date;               // When message was created
  status: MessageStatus;         // Current message state
  sources?: SourceReference[];   // Source citations (assistant only)
  error?: ErrorInfo;             // Error details if failed
}

type MessageStatus =
  | 'pending'      // User message submitted, waiting for response
  | 'sending'      // Request in flight to backend
  | 'delivered'    // Message successfully delivered
  | 'failed'       // Message delivery failed
  | 'timeout';     // Request timed out

interface SourceReference {
  id: string;                    // Unique source identifier
  content: string;               // Chunk text content (full)
  preview: string;               // First 150 characters for display
  url: string;                   // Original document URL
  title: string;                 // Document title (extracted from URL or metadata)
  chunkId: string;              // Chunk identifier from backend
  documentId: string;           // Document identifier from backend
  similarityScore: number;      // Relevance score (0.0 - 1.0)
  position: number;             // Ranking position (1-based)
}

interface ErrorInfo {
  code: string;                  // Error code (e.g., 'NETWORK_ERROR', 'TIMEOUT')
  message: string;               // User-friendly error message
  details?: string;              // Technical details for debugging
  retryable: boolean;           // Whether retry is possible
}
```

**Validation Rules**:
- `id`: Must be a valid UUID v4
- `content`: Non-empty string, max 2000 characters for user messages
- `type`: Must be either 'user' or 'assistant'
- `timestamp`: Valid Date object, not in the future
- `status`: Must be one of the defined MessageStatus values
- `sources`: Only present for assistant messages with status 'delivered'
- `similarityScore`: Number between 0.0 and 1.0
- `position`: Positive integer

**State Transitions**:
```
User Message Flow:
  [created] → pending → sending → delivered
                              ↓
                            failed
                              ↓
                           timeout

Assistant Message Flow:
  [created] → pending → delivered (with sources)
                    ↓
                  failed (with error)
```

### 2. ChatState

Global state for the chat interface.

**TypeScript Interface**:
```typescript
interface ChatState {
  messages: ChatMessage[];       // Ordered list of messages (oldest first)
  isLoading: boolean;           // Loading indicator state
  isOpen: boolean;              // Chat interface visibility
  currentRequestId: string | null; // Active request identifier
  error: ErrorState | null;     // Global error state
  config: ChatConfig;           // Configuration settings
}

interface ErrorState {
  type: ErrorType;
  message: string;
  timestamp: Date;
  recoveryAction?: RecoveryAction;
}

type ErrorType =
  | 'validation'      // Client-side validation error
  | 'network'         // Network connectivity issue
  | 'server'          // Backend error (5xx)
  | 'unavailable'     // Service unavailable (503)
  | 'timeout'         // Request timeout
  | 'unknown';        // Unexpected error

interface RecoveryAction {
  label: string;               // Button text (e.g., "Retry", "Dismiss")
  action: () => void;          // Recovery function
}

interface ChatConfig {
  apiBaseUrl: string;          // Backend API base URL
  timeoutMs: number;           // Request timeout (default: 10000)
  maxRetries: number;          // Max retry attempts (default: 3)
  retryDelayMs: number;        // Delay between retries (default: 1000)
  topK: number;                // Number of sources to retrieve (default: 5)
  enabled: boolean;            // Whether chat is enabled
}
```

**Validation Rules**:
- `messages`: Array of valid ChatMessage objects, ordered chronologically
- `isLoading`: Boolean, false if no active request
- `currentRequestId`: Valid UUID v4 or null
- `apiBaseUrl`: Valid HTTP/HTTPS URL
- `timeoutMs`: Positive integer, min 1000, max 60000
- `maxRetries`: Integer between 0 and 10
- `retryDelayMs`: Positive integer, min 100, max 10000
- `topK`: Integer between 1 and 20

### 3. APIRequest

Request payload sent to backend.

**TypeScript Interface**:
```typescript
interface AskRequest {
  query: string;                // User question
  top_k?: number;              // Optional: number of sources (default: 5)
  request_id?: string;         // Optional: client-generated request ID
}
```

**Validation Rules**:
- `query`: Non-empty string after trim, min 1 char, max 1000 chars
- `top_k`: Optional integer between 1 and 20
- `request_id`: Optional UUID v4

**Validation Function**:
```typescript
function validateAskRequest(request: AskRequest): ValidationResult {
  const errors: string[] = [];

  if (!request.query || request.query.trim().length === 0) {
    errors.push('Query cannot be empty');
  }

  if (request.query && request.query.length > 1000) {
    errors.push('Query exceeds maximum length of 1000 characters');
  }

  if (request.top_k !== undefined) {
    if (!Number.isInteger(request.top_k) || request.top_k < 1 || request.top_k > 20) {
      errors.push('top_k must be an integer between 1 and 20');
    }
  }

  return {
    isValid: errors.length === 0,
    errors
  };
}
```

### 4. APIResponse

Response payload received from backend.

**TypeScript Interface**:
```typescript
interface AskResponse {
  answer: string;               // AI-generated answer
  sources: BackendSource[];     // Retrieved document chunks
  query: string;               // Original query (echoed back)
  chunk_count: number;         // Number of sources returned
  response_time_ms: number;    // Backend processing time
}

interface BackendSource {
  id: string;                  // Point ID from Qdrant
  content: string;             // Full chunk text
  url: string;                 // Document URL
  chunk_id: string;            // Chunk identifier
  document_id: string;         // Document identifier
  similarity_score: number;    // Relevance score (0.0 - 1.0)
  position: number;            // Ranking (1-based)
}

interface APIError {
  error: boolean;              // Always true for errors
  message: string;             // Error message
  status_code: number;         // HTTP status code
  type: string;               // Error type (e.g., 'ValidationError')
  details?: any;              // Optional error details
}
```

**Validation Rules**:
- `answer`: Non-empty string
- `sources`: Array of valid BackendSource objects
- `query`: Matches the original request query
- `chunk_count`: Matches length of sources array
- `response_time_ms`: Positive number
- `similarity_score`: Number between 0.0 and 1.0
- `position`: Positive integer

**Validation Function**:
```typescript
function validateAskResponse(response: AskResponse): ValidationResult {
  const errors: string[] = [];

  if (!response.answer || response.answer.trim().length === 0) {
    errors.push('Answer cannot be empty');
  }

  if (!Array.isArray(response.sources)) {
    errors.push('Sources must be an array');
  }

  if (response.chunk_count !== response.sources.length) {
    errors.push('chunk_count does not match sources array length');
  }

  response.sources.forEach((source, index) => {
    if (source.similarity_score < 0 || source.similarity_score > 1) {
      errors.push(`Source ${index}: similarity_score must be between 0 and 1`);
    }

    if (source.position < 1) {
      errors.push(`Source ${index}: position must be a positive integer`);
    }

    if (!source.url || !isValidUrl(source.url)) {
      errors.push(`Source ${index}: invalid URL`);
    }
  });

  return {
    isValid: errors.length === 0,
    errors
  };
}
```

## State Management Schema

### ChatContext Structure

```typescript
interface ChatContextValue {
  // State
  state: ChatState;

  // Actions
  sendMessage: (query: string) => Promise<void>;
  retryMessage: (messageId: string) => Promise<void>;
  clearMessages: () => void;
  openChat: () => void;
  closeChat: () => void;
  dismissError: () => void;
}

type ChatAction =
  | { type: 'SEND_MESSAGE'; payload: { message: ChatMessage } }
  | { type: 'RECEIVE_RESPONSE'; payload: { message: ChatMessage } }
  | { type: 'MESSAGE_FAILED'; payload: { messageId: string; error: ErrorInfo } }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: ErrorState | null }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'TOGGLE_CHAT'; payload: boolean }
  | { type: 'UPDATE_CONFIG'; payload: Partial<ChatConfig> };
```

### Reducer Logic

```typescript
function chatReducer(state: ChatState, action: ChatAction): ChatState {
  switch (action.type) {
    case 'SEND_MESSAGE':
      return {
        ...state,
        messages: [...state.messages, action.payload.message],
        isLoading: true,
        error: null,
        currentRequestId: action.payload.message.id,
      };

    case 'RECEIVE_RESPONSE':
      return {
        ...state,
        messages: [...state.messages, action.payload.message],
        isLoading: false,
        currentRequestId: null,
      };

    case 'MESSAGE_FAILED':
      return {
        ...state,
        messages: state.messages.map(msg =>
          msg.id === action.payload.messageId
            ? { ...msg, status: 'failed', error: action.payload.error }
            : msg
        ),
        isLoading: false,
        currentRequestId: null,
        error: {
          type: 'server',
          message: action.payload.error.message,
          timestamp: new Date(),
        },
      };

    case 'SET_LOADING':
      return {
        ...state,
        isLoading: action.payload,
      };

    case 'SET_ERROR':
      return {
        ...state,
        error: action.payload,
        isLoading: false,
      };

    case 'CLEAR_MESSAGES':
      return {
        ...state,
        messages: [],
        error: null,
        currentRequestId: null,
      };

    case 'TOGGLE_CHAT':
      return {
        ...state,
        isOpen: action.payload,
      };

    case 'UPDATE_CONFIG':
      return {
        ...state,
        config: { ...state.config, ...action.payload },
      };

    default:
      return state;
  }
}
```

## Data Transformation Utilities

### Backend to Frontend Mapping

```typescript
function mapBackendSourceToFrontend(source: BackendSource): SourceReference {
  return {
    id: source.id,
    content: source.content,
    preview: source.content.substring(0, 150) + (source.content.length > 150 ? '...' : ''),
    url: source.url,
    title: extractTitleFromUrl(source.url),
    chunkId: source.chunk_id,
    documentId: source.document_id,
    similarityScore: source.similarity_score,
    position: source.position,
  };
}

function extractTitleFromUrl(url: string): string {
  try {
    const urlObj = new URL(url);
    const pathname = urlObj.pathname;
    const segments = pathname.split('/').filter(s => s.length > 0);
    const lastSegment = segments[segments.length - 1];

    // Remove file extension and convert dashes/underscores to spaces
    return lastSegment
      .replace(/\.[^.]*$/, '')
      .replace(/[-_]/g, ' ')
      .split(' ')
      .map(word => word.charAt(0).toUpperCase() + word.slice(1))
      .join(' ');
  } catch {
    return 'Document';
  }
}
```

### Message Factory Functions

```typescript
function createUserMessage(content: string): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'user',
    content: content.trim(),
    timestamp: new Date(),
    status: 'pending',
  };
}

function createAssistantMessage(
  response: AskResponse,
  status: MessageStatus = 'delivered'
): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'assistant',
    content: response.answer,
    timestamp: new Date(),
    status,
    sources: response.sources.map(mapBackendSourceToFrontend),
  };
}

function createErrorMessage(error: ErrorInfo): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'assistant',
    content: 'I encountered an error while processing your question.',
    timestamp: new Date(),
    status: 'failed',
    error,
  };
}
```

## Persistence & Caching

### Session Storage (Optional Enhancement)

```typescript
interface SessionData {
  messages: ChatMessage[];      // Chat history
  timestamp: Date;             // When saved
}

function saveSession(state: ChatState): void {
  const data: SessionData = {
    messages: state.messages,
    timestamp: new Date(),
  };

  sessionStorage.setItem('chat_session', JSON.stringify(data));
}

function loadSession(): SessionData | null {
  const data = sessionStorage.getItem('chat_session');
  if (!data) return null;

  try {
    const parsed = JSON.parse(data);
    // Check if session is less than 30 minutes old
    const age = Date.now() - new Date(parsed.timestamp).getTime();
    if (age > 30 * 60 * 1000) {
      sessionStorage.removeItem('chat_session');
      return null;
    }
    return parsed;
  } catch {
    return null;
  }
}
```

## Entity Relationships

```
ChatState
  ├── messages: ChatMessage[]
  │   ├── [User Message]
  │   │   └── status: MessageStatus
  │   └── [Assistant Message]
  │       ├── status: MessageStatus
  │       ├── sources: SourceReference[]
  │       └── error?: ErrorInfo
  ├── error: ErrorState
  │   └── recoveryAction?: RecoveryAction
  └── config: ChatConfig

API Flow:
  AskRequest → Backend → AskResponse
                       ↓
           mapBackendSourceToFrontend()
                       ↓
                  ChatMessage (assistant)
                       ↓
                   ChatState
```

## Validation Summary

All data entering or leaving the system must be validated:

1. **User Input** (Client-side):
   - Non-empty query
   - Length constraints (1-1000 chars)
   - No malicious content (basic sanitization)

2. **API Requests** (Client-side):
   - Valid request structure
   - Valid parameter ranges
   - Request ID generation

3. **API Responses** (Client-side):
   - Response structure validation
   - Source array validation
   - Similarity score range checks
   - URL format validation

4. **State Transitions** (Client-side):
   - Valid status transitions
   - Consistent state updates
   - Error state management

## Type Safety

All TypeScript interfaces are exported and used throughout the application:

```typescript
// types.ts
export type {
  ChatMessage,
  MessageStatus,
  SourceReference,
  ErrorInfo,
  ChatState,
  ErrorState,
  ErrorType,
  RecoveryAction,
  ChatConfig,
  AskRequest,
  AskResponse,
  BackendSource,
  APIError,
  ChatContextValue,
  ChatAction,
  SessionData,
};
```

This ensures compile-time type checking and prevents runtime type errors.
