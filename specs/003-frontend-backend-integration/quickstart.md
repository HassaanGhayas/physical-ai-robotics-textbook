# Quick Start Guide: Frontend-Backend Integration

**Feature**: 003-frontend-backend-integration
**Date**: 2025-12-17
**Phase**: Phase 1 - Implementation Guide

## Overview

This guide provides step-by-step instructions for implementing the frontend-backend integration for the RAG chatbot. Follow these steps in order to ensure a successful implementation.

## Prerequisites

Before starting implementation:

1. ✅ **Backend API Running**: The FastAPI backend from feature 002-rag-agent must be deployed and accessible
2. ✅ **Qdrant Collection**: The `rag_embedding` collection must exist and contain embedded documents
3. ✅ **Environment Variables**: API keys for Cohere and OpenAI must be configured
4. ✅ **Docusaurus Site**: The frontend Docusaurus site must be running locally
5. ✅ **Node.js & npm**: Version 18+ installed
6. ✅ **TypeScript**: Understanding of TypeScript and React

## Phase 1: Setup & Configuration

### Step 1.1: Install Dependencies

```bash
# Navigate to the project root
cd my-web

# Install required dependencies
npm install axios
npm install --save-dev @types/node

# Optional: Install testing dependencies
npm install --save-dev @testing-library/react @testing-library/jest-dom vitest
```

### Step 1.2: Configure Environment Variables

Create `.env.local` file in the project root:

```bash
# .env.local
REACT_APP_API_URL=http://localhost:8000
REACT_APP_CHATBOT_ENABLED=true
REACT_APP_MAX_RETRIES=3
REACT_APP_TIMEOUT_MS=10000
```

Update `docusaurus.config.js` to use environment variables:

```javascript
// docusaurus.config.js
module.exports = {
  // ... existing config
  customFields: {
    ragApiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
    chatbotEnabled: process.env.REACT_APP_CHATBOT_ENABLED === 'true',
    maxRetries: parseInt(process.env.REACT_APP_MAX_RETRIES || '3', 10),
    timeoutMs: parseInt(process.env.REACT_APP_TIMEOUT_MS || '10000', 10),
  },
};
```

### Step 1.3: Create Directory Structure

```bash
# Create component directories
mkdir -p src/components/ChatBot
mkdir -p src/components/ChatBot/__tests__
mkdir -p src/lib/api
mkdir -p src/types
```

Expected structure:
```
src/
├── components/
│   └── ChatBot/
│       ├── ChatBot.tsx
│       ├── ChatInterface.tsx
│       ├── MessageList.tsx
│       ├── InputBar.tsx
│       ├── SourceList.tsx
│       ├── ErrorBoundary.tsx
│       ├── LoadingIndicator.tsx
│       ├── styles.module.css
│       └── __tests__/
│           └── ChatBot.test.tsx
├── lib/
│   └── api/
│       ├── client.ts
│       ├── types.ts
│       └── validators.ts
└── types/
    └── chat.ts
```

## Phase 2: Core Implementation

### Step 2.1: Define TypeScript Types

Create `src/types/chat.ts` (use definitions from data-model.md):

```typescript
// src/types/chat.ts
export interface ChatMessage {
  id: string;
  type: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  status: MessageStatus;
  sources?: SourceReference[];
  error?: ErrorInfo;
}

export type MessageStatus =
  | 'pending'
  | 'sending'
  | 'delivered'
  | 'failed'
  | 'timeout';

// ... (copy remaining types from data-model.md)
```

### Step 2.2: Implement API Client

Create `src/lib/api/client.ts`:

```typescript
// src/lib/api/client.ts
import axios, { AxiosInstance, AxiosError } from 'axios';
import type {
  AskRequest,
  AskResponse,
  ApiResult,
  ApiError,
  ApiClientConfig,
  RagApiClient
} from './types';

export function createApiClient(config: ApiClientConfig): RagApiClient {
  const axiosInstance: AxiosInstance = axios.create({
    baseURL: config.baseUrl,
    timeout: config.timeout || 10000,
    headers: {
      'Content-Type': 'application/json',
      ...config.headers,
    },
  });

  // Request/response interceptors
  axiosInstance.interceptors.request.use(
    (config) => {
      console.log(`[API] ${config.method?.toUpperCase()} ${config.url}`);
      return config;
    },
    (error) => Promise.reject(error)
  );

  axiosInstance.interceptors.response.use(
    (response) => {
      console.log(`[API] Response:`, response.status);
      return response;
    },
    (error) => {
      console.error(`[API] Error:`, error.message);
      return Promise.reject(error);
    }
  );

  // Implement ask method
  async function ask(request: AskRequest): Promise<ApiResult<AskResponse>> {
    try {
      const response = await axiosInstance.post<AskResponse>('/ask', request);
      return { success: true, data: response.data };
    } catch (error) {
      return { success: false, error: handleApiError(error as AxiosError) };
    }
  }

  // Implement other methods...

  return {
    ask,
    checkHealth: async () => { /* ... */ },
    cancelRequest: (requestId: string) => { /* ... */ },
    updateConfig: (newConfig: Partial<ApiClientConfig>) => { /* ... */ },
  };
}

function handleApiError(error: AxiosError): ApiError {
  // Error handling logic
  // ... (implement based on error types)
}
```

### Step 2.3: Create Chat Context

Create `src/components/ChatBot/ChatContext.tsx`:

```typescript
// src/components/ChatBot/ChatContext.tsx
import React, { createContext, useContext, useReducer, useCallback } from 'react';
import type { ChatState, ChatAction, ChatContextValue } from '@site/src/types/chat';
import { createApiClient } from '@site/src/lib/api/client';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const ChatContext = createContext<ChatContextValue | undefined>(undefined);

export function ChatProvider({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();
  const apiClient = createApiClient({
    baseUrl: siteConfig.customFields?.ragApiUrl as string,
    timeout: siteConfig.customFields?.timeoutMs as number,
    maxRetries: siteConfig.customFields?.maxRetries as number,
  });

  const [state, dispatch] = useReducer(chatReducer, initialState);

  const sendMessage = useCallback(async (query: string) => {
    // Implementation
  }, []);

  // ... other methods

  return (
    <ChatContext.Provider value={{ state, sendMessage, /* ... */ }}>
      {children}
    </ChatContext.Provider>
  );
}

export function useChatContext() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }
  return context;
}
```

### Step 2.4: Create Main Chat Component

Create `src/components/ChatBot/ChatBot.tsx`:

```typescript
// src/components/ChatBot/ChatBot.tsx
import React, { lazy, Suspense, useState } from 'react';
import styles from './styles.module.css';

const ChatInterface = lazy(() => import('./ChatInterface'));

export default function ChatBot(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);

  return (
    <>
      {/* Floating chat button */}
      <button
        className={styles.chatButton}
        onClick={() => setIsOpen(true)}
        aria-label="Open chat"
      >
        💬 Ask a Question
      </button>

      {/* Chat modal */}
      {isOpen && (
        <Suspense fallback={<LoadingSpinner />}>
          <ChatInterface onClose={() => setIsOpen(false)} />
        </Suspense>
      )}
    </>
  );
}
```

### Step 2.5: Create Chat Interface Component

Create `src/components/ChatBot/ChatInterface.tsx`:

```typescript
// src/components/ChatBot/ChatInterface.tsx
import React from 'react';
import { ChatProvider, useChatContext } from './ChatContext';
import MessageList from './MessageList';
import InputBar from './InputBar';
import ErrorBoundary from './ErrorBoundary';
import styles from './styles.module.css';

interface ChatInterfaceProps {
  onClose: () => void;
}

function ChatInterfaceContent({ onClose }: ChatInterfaceProps) {
  const { state, sendMessage } = useChatContext();

  return (
    <div className={styles.chatModal}>
      <div className={styles.chatHeader}>
        <h3>Ask the Knowledge Base</h3>
        <button onClick={onClose} aria-label="Close chat">×</button>
      </div>

      <MessageList messages={state.messages} isLoading={state.isLoading} />

      <InputBar
        onSend={sendMessage}
        disabled={state.isLoading}
      />
    </div>
  );
}

export default function ChatInterface(props: ChatInterfaceProps) {
  return (
    <ErrorBoundary>
      <ChatProvider>
        <ChatInterfaceContent {...props} />
      </ChatProvider>
    </ErrorBoundary>
  );
}
```

## Phase 3: UI Components

### Step 3.1: Create Message List Component

Create `src/components/ChatBot/MessageList.tsx` with:
- Message rendering (user and assistant)
- Source citation display
- Loading indicators
- Empty state

### Step 3.2: Create Input Bar Component

Create `src/components/ChatBot/InputBar.tsx` with:
- Text input with auto-resize
- Submit button
- Keyboard shortcuts (Enter to send)
- Input validation

### Step 3.3: Create Source List Component

Create `src/components/ChatBot/SourceList.tsx` with:
- Collapsible accordion
- Source preview (150 chars)
- Similarity scores
- Clickable links

### Step 3.4: Add Styling

Create `src/components/ChatBot/styles.module.css`:

```css
/* styles.module.css */
.chatButton {
  position: fixed;
  bottom: 2rem;
  right: 2rem;
  padding: 1rem 1.5rem;
  background: var(--ifm-color-primary);
  color: white;
  border: none;
  border-radius: 2rem;
  cursor: pointer;
  font-size: 1rem;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  transition: transform 0.2s;
  z-index: 1000;
}

.chatButton:hover {
  transform: scale(1.05);
}

.chatModal {
  position: fixed;
  bottom: 2rem;
  right: 2rem;
  width: 400px;
  max-height: 600px;
  background: var(--ifm-background-color);
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 0.5rem;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
  display: flex;
  flex-direction: column;
  z-index: 1001;
}

/* ... more styles */
```

## Phase 4: Integration with Docusaurus

### Step 4.1: Add ChatBot to Root

Update `src/theme/Root.tsx` (create if doesn't exist):

```typescript
// src/theme/Root.tsx
import React from 'react';
import ChatBot from '@site/src/components/ChatBot/ChatBot';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}
```

### Step 4.2: Configure Backend CORS

Update backend `main.py` to allow frontend origin:

```python
# backend/main.py
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://hassaanghayas.github.io"
    ],
    allow_methods=["POST", "OPTIONS", "GET"],
    allow_headers=["Content-Type"],
    allow_credentials=False,
    max_age=600
)
```

## Phase 5: Testing

### Step 5.1: Write Component Tests

Create `src/components/ChatBot/__tests__/ChatBot.test.tsx`:

```typescript
// __tests__/ChatBot.test.tsx
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatBot from '../ChatBot';

describe('ChatBot', () => {
  it('opens chat interface when button is clicked', () => {
    render(<ChatBot />);
    const button = screen.getByLabelText('Open chat');
    fireEvent.click(button);
    expect(screen.getByText('Ask the Knowledge Base')).toBeInTheDocument();
  });

  // More tests...
});
```

### Step 5.2: Manual Testing Checklist

- [ ] Chat button appears on all pages
- [ ] Modal opens and closes correctly
- [ ] User can type and submit questions
- [ ] Loading indicator shows during request
- [ ] Answers display with proper formatting
- [ ] Sources are expandable and clickable
- [ ] Error messages display appropriately
- [ ] Retry functionality works
- [ ] Works on mobile devices
- [ ] Keyboard navigation works

### Step 5.3: Integration Testing

Test with actual backend:

```bash
# Start backend
cd backend
python main.py

# Start frontend in another terminal
cd my-web
npm start

# Test scenarios:
# 1. Submit valid question
# 2. Submit empty question
# 3. Network error (stop backend)
# 4. Long response time
```

## Phase 6: Deployment

### Step 6.1: Build for Production

```bash
# Build the site
npm run build

# Test production build locally
npm run serve
```

### Step 6.2: Deploy Backend

Ensure backend is deployed and accessible at the production URL:
- Update environment variables with production API URL
- Verify CORS allows production origin
- Test `/health` endpoint

### Step 6.3: Deploy Frontend

```bash
# Deploy to GitHub Pages
npm run deploy
```

## Troubleshooting

### Issue: CORS Error

**Solution**: Check backend CORS configuration includes frontend origin.

```python
allow_origins=["https://hassaanghayas.github.io"]
```

### Issue: Timeout Errors

**Solution**: Increase timeout or optimize backend response time.

```typescript
// Increase timeout in config
timeoutMs: 15000  // 15 seconds
```

### Issue: Chat Not Appearing

**Solution**: Verify Root.tsx is being used and chatbot is enabled.

```bash
# Check if Root.tsx exists
ls src/theme/Root.tsx

# Verify config
echo $REACT_APP_CHATBOT_ENABLED
```

## Next Steps

After successful implementation:

1. ✅ Monitor API response times and error rates
2. ✅ Collect user feedback on chat experience
3. ✅ Optimize bundle size if needed
4. ✅ Add analytics tracking
5. ✅ Consider adding features:
   - Chat history persistence
   - Markdown rendering in answers
   - Code syntax highlighting
   - Multi-turn conversations

## Resources

- **API Contract**: `specs/003-frontend-backend-integration/contracts/api-contract.yaml`
- **Data Models**: `specs/003-frontend-backend-integration/data-model.md`
- **Research**: `specs/003-frontend-backend-integration/research.md`
- **Docusaurus Docs**: https://docusaurus.io/docs
- **React Testing Library**: https://testing-library.com/react
- **Axios Docs**: https://axios-http.com/docs/intro
