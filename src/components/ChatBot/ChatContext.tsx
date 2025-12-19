/**
 * Chat Context with React useReducer for state management
 * Provides chat state and actions to all chat components
 */

import React, { createContext, useContext, useReducer, useCallback, useMemo } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import type {
  ChatState,
  ChatAction,
  ChatContextValue,
  ChatMessage,
  ChatConfig,
  ErrorState,
} from '../../types/chat';
import { createUserMessage } from '../../types/chat';
import { createApiClient } from '../../lib/api/client';
import type { ApiClientConfig, AskResponse, BackendSource } from '../../lib/api/types';
import type { SourceReference } from '../../types/chat';
import { extractTitleFromUrl } from '../../types/chat';

/**
 * Map backend source to frontend SourceReference
 */
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

/**
 * Create assistant message from API response
 */
function createAssistantMessage(response: AskResponse): ChatMessage {
  return {
    id: crypto.randomUUID(),
    type: 'assistant',
    content: response.answer,
    timestamp: new Date(),
    status: 'delivered',
    sources: response.sources.map(mapBackendSourceToFrontend),
  };
}

/**
 * Chat reducer implementing all state transitions
 */
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

/**
 * Create Chat Context
 */
const ChatContext = createContext<ChatContextValue | undefined>(undefined);

/**
 * Chat Provider component
 */
export function ChatProvider({ children }: { children: React.ReactNode }) {
  const { siteConfig } = useDocusaurusContext();

  // Initialize config from Docusaurus custom fields
  const initialConfig: ChatConfig = {
    apiBaseUrl: (siteConfig.customFields?.ragApiUrl as string) || 'http://localhost:8000',
    timeoutMs: (siteConfig.customFields?.timeoutMs as number) || 10000,
    maxRetries: (siteConfig.customFields?.maxRetries as number) || 3,
    retryDelayMs: 1000,
    topK: 5,
    enabled: (siteConfig.customFields?.chatbotEnabled as boolean) !== false,
  };

  // Initialize state
  const initialState: ChatState = {
    messages: [],
    isLoading: false,
    isOpen: false,
    currentRequestId: null,
    error: null,
    config: initialConfig,
  };

  const [state, dispatch] = useReducer(chatReducer, initialState);

  // Create API client instance (memoized)
  const apiClient = useMemo(() => {
    const config: ApiClientConfig = {
      baseUrl: state.config.apiBaseUrl,
      timeout: state.config.timeoutMs,
      maxRetries: state.config.maxRetries,
      retryDelay: state.config.retryDelayMs,
      exponentialBackoff: true,
    };
    return createApiClient(config);
  }, [state.config.apiBaseUrl, state.config.timeoutMs, state.config.maxRetries, state.config.retryDelayMs]);

  /**
   * Send a message
   */
  const sendMessage = useCallback(async (query: string) => {
    // Create user message
    const userMessage = createUserMessage(query);

    // Add user message to state
    dispatch({
      type: 'SEND_MESSAGE',
      payload: { message: userMessage },
    });

    try {
      // Call API
      const result = await apiClient.ask({
        query,
        top_k: state.config.topK,
        request_id: userMessage.id,
      });

      if (result.success) {
        // Check if there are no relevant results
        if (!result.data.sources || result.data.sources.length === 0) {
          // Create assistant message with helpful suggestion
          const noResultsMessage: ChatMessage = {
            id: crypto.randomUUID(),
            type: 'assistant',
            content: result.data.answer || "I couldn't find relevant information about that topic in the textbook. Try rephrasing your question or asking about Physical AI and Humanoid Robotics topics covered in this book.",
            timestamp: new Date(),
            status: 'delivered',
            sources: [],
          };

          dispatch({
            type: 'RECEIVE_RESPONSE',
            payload: { message: noResultsMessage },
          });
        } else {
          // Create assistant message with sources
          const assistantMessage = createAssistantMessage(result.data);

          // Add assistant message to state
          dispatch({
            type: 'RECEIVE_RESPONSE',
            payload: { message: assistantMessage },
          });
        }
      } else {
        // Handle error
        dispatch({
          type: 'MESSAGE_FAILED',
          payload: {
            messageId: userMessage.id,
            error: {
              code: result.error.code,
              message: result.error.message,
              details: result.error.details,
              retryable: result.error.retryable,
            },
          },
        });
      }
    } catch (error) {
      // Unexpected error
      dispatch({
        type: 'MESSAGE_FAILED',
        payload: {
          messageId: userMessage.id,
          error: {
            code: 'UNKNOWN_ERROR',
            message: 'An unexpected error occurred',
            retryable: false,
          },
        },
      });
    }
  }, [apiClient, state.config.topK]);

  /**
   * Retry a failed message
   */
  const retryMessage = useCallback(async (messageId: string) => {
    const message = state.messages.find(m => m.id === messageId);
    if (!message || message.type !== 'user') {
      return;
    }

    // Re-send the message
    await sendMessage(message.content);
  }, [state.messages, sendMessage]);

  /**
   * Clear all messages
   */
  const clearMessages = useCallback(() => {
    dispatch({ type: 'CLEAR_MESSAGES' });
  }, []);

  /**
   * Open chat interface
   */
  const openChat = useCallback(() => {
    dispatch({ type: 'TOGGLE_CHAT', payload: true });
  }, []);

  /**
   * Close chat interface
   */
  const closeChat = useCallback(() => {
    dispatch({ type: 'TOGGLE_CHAT', payload: false });
  }, []);

  /**
   * Dismiss global error
   */
  const dismissError = useCallback(() => {
    dispatch({ type: 'SET_ERROR', payload: null });
  }, []);

  // Create context value
  const contextValue: ChatContextValue = {
    state,
    sendMessage,
    retryMessage,
    clearMessages,
    openChat,
    closeChat,
    dismissError,
  };

  return (
    <ChatContext.Provider value={contextValue}>
      {children}
    </ChatContext.Provider>
  );
}

/**
 * Hook to use chat context
 */
export function useChatContext(): ChatContextValue {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }
  return context;
}
