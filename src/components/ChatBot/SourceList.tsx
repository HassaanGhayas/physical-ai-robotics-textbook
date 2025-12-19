/**
 * Source List Component
 * Displays source citations in collapsible accordion
 */

import React, { useState, memo } from 'react';
import type { SourceReference } from '../../types/chat';
import styles from './styles.module.css';

interface SourceListProps {
  sources: SourceReference[];
}

const SourceList = memo(function SourceList({ sources }: SourceListProps): JSX.Element | null {
  const [isExpanded, setIsExpanded] = useState(false);

  if (!sources || sources.length === 0) {
    return null;
  }

  return (
    <div className={styles.sourceList}>
      <button
        className={styles.sourceToggle}
        onClick={() => setIsExpanded(!isExpanded)}
        aria-expanded={isExpanded}
        aria-label={`${isExpanded ? 'Collapse' : 'Expand'} sources`}
      >
        <span>Sources ({sources.length})</span>
        <span className={styles.toggleIcon}>{isExpanded ? '▼' : '▶'}</span>
      </button>

      {isExpanded && (
        <div className={styles.sourceItems}>
          {sources.map((source, index) => (
            <div key={source.id} className={styles.sourceItem}>
              <div className={styles.sourceHeader}>
                <span className={styles.sourcePosition}>[{source.position}]</span>
                <span className={styles.sourceTitle}>{source.title}</span>
                <span
                  className={`${styles.sourceScore} ${
                    source.similarityScore > 0.7
                      ? styles.scoreHigh
                      : source.similarityScore > 0.5
                      ? styles.scoreMedium
                      : styles.scoreLow
                  }`}
                >
                  Score: {source.similarityScore.toFixed(2)}
                </span>
              </div>

              <div className={styles.sourcePreview}>
                "{source.preview}"
              </div>

              <a
                href={source.url}
                target="_blank"
                rel="noopener noreferrer"
                className={styles.sourceLink}
                title={`${source.title} (Relevance: ${source.similarityScore.toFixed(2)})`}
                aria-label={`Open source document: ${source.title} in new tab`}
              >
                View full document →
              </a>
            </div>
          ))}
        </div>
      )}
    </div>
  );
});

export default SourceList;
