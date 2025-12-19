/**
 * Error Message Component
 * Displays error messages with retry option
 */

import React, { memo } from 'react';
import styles from './styles.module.css';

interface ErrorProps {
  code?: string;
  message: string;
  retryable?: boolean;
  details?: string;
}

interface ErrorMessageProps {
  error: ErrorProps;
  onRetry?: () => void;
}

const ErrorMessage = memo(function ErrorMessage({ error, onRetry }: ErrorMessageProps): JSX.Element {
  return (
    <div className={styles.errorMessage}>
      <div className={styles.errorContent}>
        <span className={styles.errorIcon}>⚠️</span>
        <div className={styles.errorText}>
          <p className={styles.errorMessageText}>{error.message}</p>
          {error.details && (
            <p className={styles.errorDetails}>{error.details}</p>
          )}
        </div>
      </div>

      {error.retryable && onRetry && (
        <button
          className={styles.retryButton}
          onClick={onRetry}
          aria-label="Retry request"
        >
          Try Again
        </button>
      )}
    </div>
  );
});

export default ErrorMessage;
