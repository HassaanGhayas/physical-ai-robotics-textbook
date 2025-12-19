/**
 * Input Bar Component
 * Text input for submitting questions with validation
 */

import React, { useState, useRef, useEffect, KeyboardEvent, memo } from 'react';
import styles from './styles.module.css';

interface InputBarProps {
  onSend: (query: string) => void;
  disabled: boolean;
}

const InputBar = memo(function InputBar({ onSend, disabled }: InputBarProps): JSX.Element {
  const [input, setInput] = useState('');
  const [validationError, setValidationError] = useState<string | null>(null);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${Math.min(textareaRef.current.scrollHeight, 120)}px`;
    }
  }, [input]);

  // Handle input change
  const handleChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setInput(e.target.value);
    setValidationError(null);
  };

  // Handle submit
  const handleSubmit = () => {
    const trimmed = input.trim();

    // Validate input
    if (!trimmed) {
      setValidationError('Please enter a question');
      return;
    }

    if (trimmed.length > 1000) {
      setValidationError('Question is too long (max 1000 characters)');
      return;
    }

    // Clear validation error
    setValidationError(null);

    // Send message
    onSend(trimmed);

    // Clear input
    setInput('');

    // Reset textarea height
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
  };

  // Handle Enter key (submit) vs Shift+Enter (newline)
  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  // Character count
  const charCount = input.length;
  const isOverLimit = charCount > 1000;

  return (
    <div className={styles.inputBar}>
      {validationError && (
        <div className={styles.validationError}>
          {validationError}
        </div>
      )}

      <div className={styles.inputContainer}>
        <textarea
          ref={textareaRef}
          className={styles.textarea}
          value={input}
          onChange={handleChange}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question about Physical AI & Robotics..."
          disabled={disabled}
          rows={1}
          aria-label="Chat input"
        />

        <div className={styles.inputActions}>
          <span className={`${styles.charCounter} ${isOverLimit ? styles.overLimit : ''}`}>
            {charCount}/1000
          </span>

          <button
            className={styles.sendButton}
            onClick={handleSubmit}
            disabled={disabled || !input.trim()}
            aria-label="Send message"
          >
            Send
          </button>
        </div>
      </div>
    </div>
  );
});

export default InputBar;
