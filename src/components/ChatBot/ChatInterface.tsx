/**
 * Chat Interface Component
 * Main chat UI with header, messages, and input
 */

import React, { useEffect, useRef, useCallback } from 'react';
import { ChatProvider, useChatContext } from './ChatContext';
import MessageList from './MessageList';
import InputBar from './InputBar';
import ErrorBoundary from './ErrorBoundary';
import styles from './styles.module.css';

interface ChatInterfaceProps {
  onClose: () => void;
}

/**
 * Chat Interface Content (must be wrapped by ChatProvider)
 */
function ChatInterfaceContent({ onClose }: ChatInterfaceProps) {
  const { state, sendMessage, retryMessage, clearMessages, dismissError } = useChatContext();
  const modalRef = useRef<HTMLDivElement>(null);
  const firstFocusableRef = useRef<HTMLButtonElement>(null);

  // Handle ESC key to close chat
  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      onClose();
    }
  }, [onClose]);

  // Focus trap - keep focus within modal
  const handleFocusTrap = useCallback((e: KeyboardEvent) => {
    if (e.key !== 'Tab') return;

    const modal = modalRef.current;
    if (!modal) return;

    const focusableElements = modal.querySelectorAll<HTMLElement>(
      'button:not([disabled]), textarea:not([disabled]), a[href], input:not([disabled]), [tabindex]:not([tabindex="-1"])'
    );

    if (focusableElements.length === 0) return;

    const firstElement = focusableElements[0];
    const lastElement = focusableElements[focusableElements.length - 1];

    if (e.shiftKey) {
      // Shift+Tab: if on first element, go to last
      if (document.activeElement === firstElement) {
        e.preventDefault();
        lastElement.focus();
      }
    } else {
      // Tab: if on last element, go to first
      if (document.activeElement === lastElement) {
        e.preventDefault();
        firstElement.focus();
      }
    }
  }, []);

  // Set up keyboard listeners
  useEffect(() => {
    document.addEventListener('keydown', handleKeyDown);
    document.addEventListener('keydown', handleFocusTrap);

    // Focus first focusable element when modal opens
    // Use a small timeout to ensure the DOM is ready
    const timeoutId = setTimeout(() => {
      const modal = modalRef.current;
      if (modal) {
        const firstFocusable = modal.querySelector<HTMLElement>(
          'textarea:not([disabled]), button:not([disabled])'
        );
        firstFocusable?.focus();
      }
    }, 100);

    return () => {
      document.removeEventListener('keydown', handleKeyDown);
      document.removeEventListener('keydown', handleFocusTrap);
      clearTimeout(timeoutId);
    };
  }, [handleKeyDown, handleFocusTrap]);

  return (
    <div
      ref={modalRef}
      className={styles.chatModal}
      role="dialog"
      aria-modal="true"
      aria-labelledby="chat-title"
    >
      {/* Header */}
      <div className={styles.chatHeader}>
        <div className={styles.headerContent}>
          <h3 id="chat-title" className={styles.chatTitle}>Ask the Knowledge Base</h3>
          <p className={styles.chatSubtitle}>Physical AI & Humanoid Robotics</p>
        </div>

        <div className={styles.headerActions}>
          {state.messages.length > 0 && (
            <button
              className={styles.clearButton}
              onClick={clearMessages}
              aria-label="Clear messages"
              title="Clear all messages"
            >
              🗑️
            </button>
          )}

          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chat"
          >
            ×
          </button>
        </div>
      </div>

      {/* Global error banner */}
      {state.error && (
        <div className={styles.errorBanner}>
          <span className={styles.errorIcon}>⚠️</span>
          <span className={styles.errorMessage}>{state.error.message}</span>
          <button
            className={styles.dismissButton}
            onClick={dismissError}
            aria-label="Dismiss error"
          >
            ×
          </button>
        </div>
      )}

      {/* Messages */}
      <div className={styles.chatBody}>
        <MessageList
          messages={state.messages}
          isLoading={state.isLoading}
          onRetry={retryMessage}
        />
      </div>

      {/* Input */}
      <div className={styles.chatFooter}>
        <InputBar onSend={sendMessage} disabled={state.isLoading} />
      </div>
    </div>
  );
}

/**
 * Chat Interface with Provider and Error Boundary
 */
export default function ChatInterface(props: ChatInterfaceProps) {
  return (
    <ErrorBoundary>
      <ChatProvider>
        <ChatInterfaceContent {...props} />
      </ChatProvider>
    </ErrorBoundary>
  );
}
