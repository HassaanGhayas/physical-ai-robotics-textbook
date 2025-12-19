/**
 * Chat Bot Component
 * Main entry point with floating button and lazy-loaded interface
 */

import React, { lazy, Suspense, useState } from 'react';
import LoadingIndicator from './LoadingIndicator';
import styles from './styles.module.css';

// Lazy load the chat interface for better performance
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
        title="Ask a question"
      >
        💬 Ask a Question
      </button>

      {/* Chat interface modal */}
      {isOpen && (
        <Suspense fallback={
          <div className={styles.chatModal}>
            <div className={styles.loadingFallback}>
              <LoadingIndicator />
            </div>
          </div>
        }>
          <ChatInterface onClose={() => setIsOpen(false)} />
        </Suspense>
      )}
    </>
  );
}
