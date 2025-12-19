/**
 * Chat data models for RAG chatbot frontend
 * Based on data-model.md from specs/003-frontend-backend-integration
 */

/**
 * Message status type
 */
export type MessageStatus =
  | 'pending'      // User message submitted, waiting for response
  | 'sending'      // Request in flight to backend
  | 'delivered'    // Message successfully delivered
  | 'failed'       // Message delivery failed
  | 'timeout';     // Request timed out

/**
 * Error information for failed messages
 */
export interface ErrorInfo {
  /** Error code (e.g., 'NETWORK_ERROR', 'TIMEOUT') */
  code: string;

  /** User-friendly error message */
  message: string;

  /** Technical details for debugging */
  details?: string;

  /** Whether retry is possible */
  retryable: boolean;
}

/**
 * Source reference from retrieved documents
 */
export interface SourceReference {
  /** Unique source identifier */
  id: string;

  /** Chunk text content (full) */
  content: string;

  /** First 150 characters for display */
  preview: string;

  /** Original document URL */
  url: string;

  /** Document title (extracted from URL or metadata) */
  title: string;

  /** Chunk identifier from backend */
  chunkId: string;

  /** Document identifier from backend */
  documentId: string;

  /** Relevance score (0.0 - 1.0) */
  similarityScore: number;

  /** Ranking position (1-based) */
  position: number;
}

/**
 * Chat message (user question or AI response)
 */
export interface ChatMessage {
  /** Unique message identifier (UUID) */
  id: string;

  /** Message sender type */
  type: 'user' | 'assistant';

  /** Message text content */
  content: string;

  /** When message was created */
  timestamp: Date;

  /** Current message state */
  status: MessageStatus;

  /** Source citations (assistant only) */
  sources?: SourceReference[];

  /** Error details if failed */
  error?: ErrorInfo;
}

/**
 * Error type categories
 */
export type ErrorType =
  | 'validation'      // Client-side validation error
  | 'network'         // Network connectivity issue
  | 'server'          // Backend error (5xx)
  | 'unavailable'     // Service unavailable (503)
  | 'timeout'         // Request timeout
  | 'unknown';        // Unexpected error

/**
 * Recovery action for errors
 */
export interface RecoveryAction {
  /** Button text (e.g., "Retry", "Dismiss") */
  label: string;

  /** Recovery function */
  action: () => void;
}

/**
 * Global error state
 */
export interface ErrorState {
  /** Error type */
  type: ErrorType;

  /** Error message */
  message: string;

  /** When error occurred */
  timestamp: Date;

  /** Optional recovery action */
  recoveryAction?: RecoveryAction;
}

/**
 * Chat configuration
 */
export interface ChatConfig {
  /** Backend API base URL */
  apiBaseUrl: string;

  /** Request timeout (default: 10000) */
  timeoutMs: number;

  /** Max retry attempts (default: 3) */
  maxRetries: number;

  /** Delay between retries (default: 1000) */
  retryDelayMs: number;

  /** Number of sources to retrieve (default: 5) */
  topK: number;

  /** Whether chat is enabled */
  enabled: boolean;
}

/**
 * Global chat state
 */
export interface ChatState {
  /** Ordered list of messages (oldest first) */
  messages: ChatMessage[];

  /** Loading indicator state */
  isLoading: boolean;

  /** Chat interface visibility */
  isOpen: boolean;

  /** Active request identifier */
  currentRequestId: string | null;

  /** Global error state */
  error: ErrorState | null;

  /** Configuration settings */
  config: ChatConfig;
}

/**
 * Chat action types
 */
export type ChatAction =
  | { type: 'SEND_MESSAGE'; payload: { message: ChatMessage } }
  | { type: 'RECEIVE_RESPONSE'; payload: { message: ChatMessage } }
  | { type: 'MESSAGE_FAILED'; payload: { messageId: string; error: ErrorInfo } }
  | { type: 'SET_LOADING'; payload: boolean }
  | { type: 'SET_ERROR'; payload: ErrorState | null }
  | { type: 'CLEAR_MESSAGES' }
  | { type: 'TOGGLE_CHAT'; payload: boolean }
  | { type: 'UPDATE_CONFIG'; payload: Partial<ChatConfig> };

/**
 * Chat context value (state + actions)
 */
export interface ChatContextValue {
  /** Current state */
  state: ChatState;

  /** Send a message */
  sendMessage: (query: string) => Promise<void>;

  /** Retry a failed message */
  retryMessage: (messageId: string) => Promise<void>;

  /** Clear all messages */
  clearMessages: () => void;

  /** Open chat interface */
  openChat: () => void;

  /** Close chat interface */
  closeChat: () => void;

  /** Dismiss global error */
  dismissError: () => void;
}

/**
 * Session data for persistence
 */
export interface SessionData {
  /** Chat history */
  messages: ChatMessage[];

  /** When saved */
  timestamp: Date;
}

/**
 * Utility: Create user message
 */
export function createUserMessage(content: string): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'user',
    content: content.trim(),
    timestamp: new Date(),
    status: 'pending',
  };
}

/**
 * Utility: Extract title from URL
 */
export function extractTitleFromUrl(url: string): string {
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
