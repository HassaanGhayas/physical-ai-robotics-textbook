/**
 * Message List Component
 * Displays chat messages with auto-scrolling and error handling
 */

import React, { useEffect, useRef, memo } from 'react';
import type { ChatMessage } from '../../types/chat';
import SourceList from './SourceList';
import LoadingIndicator from './LoadingIndicator';
import ErrorMessage from './ErrorMessage';
import styles from './styles.module.css';

interface MessageListProps {
  messages: ChatMessage[];
  isLoading: boolean;
  onRetry?: (messageId: string) => void;
}

const MessageList = memo(function MessageList({ messages, isLoading, onRetry }: MessageListProps): JSX.Element {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, isLoading]);

  // Format timestamp
  const formatTime = (date: Date): string => {
    return new Date(date).toLocaleTimeString('en-US', {
      hour: '2-digit',
      minute: '2-digit',
    });
  };

  return (
    <div className={styles.messageList}>
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          <p>Ask a question about Physical AI & Humanoid Robotics!</p>
          <p className={styles.emptyHint}>
            Try asking: "What is physical AI?" or "How do humanoid robots work?"
          </p>
        </div>
      )}

      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.messageContainer} ${
            message.type === 'user' ? styles.userMessage : styles.assistantMessage
          }`}
        >
          <div className={styles.messageBubble}>
            <div className={styles.messageContent}>{message.content}</div>

            {message.type === 'assistant' && message.sources && message.sources.length > 0 && (
              <SourceList sources={message.sources} />
            )}

            {message.error && (
              <ErrorMessage
                error={message.error}
                onRetry={onRetry ? () => onRetry(message.id) : undefined}
              />
            )}
          </div>

          <div className={styles.messageTimestamp}>
            {formatTime(message.timestamp)}
          </div>
        </div>
      ))}

      {isLoading && <LoadingIndicator />}

      <div ref={messagesEndRef} />
    </div>
  );
});

export default MessageList;
