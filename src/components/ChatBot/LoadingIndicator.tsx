/**
 * Loading Indicator Component
 * Displays animated typing indicator while waiting for response
 */

import React, { memo } from 'react';
import styles from './styles.module.css';

const LoadingIndicator = memo(function LoadingIndicator(): JSX.Element {
  return (
    <div className={styles.loadingIndicator}>
      <div className={styles.typingDots}>
        <span className={styles.dot}></span>
        <span className={styles.dot}></span>
        <span className={styles.dot}></span>
      </div>
      <span className={styles.loadingText}>AI is thinking...</span>
    </div>
  );
});

export default LoadingIndicator;
