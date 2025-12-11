import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './CodeExamples.module.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

// Define the BrowserOnly component for syntax highlighting
const SyntaxHighlightedCode = ({ code, language }: { code: string; language: string }) => {
  return (
    <BrowserOnly>
      {() => {
        // Dynamically import Prism for syntax highlighting
        const Prism = require('prismjs');
        require('prismjs/components/prism-python');
        require('prismjs/components/prism-javascript');
        require('prismjs/components/prism-typescript');
        require('prismjs/components/prism-bash');
        require('prismjs/components/prism-json');
        require('prismjs/components/prism-yaml');

        const highlightedCode = Prism.highlight(
          code,
          Prism.languages[language] || Prism.languages.text,
          language
        );

        return (
          <code
            className={`language-${language}`}
            dangerouslySetInnerHTML={{ __html: highlightedCode }}
          />
        );
      }}
    </BrowserOnly>
  );
};

type CodeExampleProps = {
  title: string;
  description: string;
  code: string;
  language: string;
  copyable?: boolean;
  expandable?: boolean;
  maxHeight?: string;
};

export default function CodeExamples({
  title,
  description,
  code,
  language = 'python',
  copyable = true,
  expandable = true,
  maxHeight = '400px'
}: CodeExampleProps): JSX.Element {
  const [expanded, setExpanded] = useState(false);
  const [copied, setCopied] = useState(false);

  const copyToClipboard = () => {
    navigator.clipboard.writeText(code).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  };

  const displayedCode = expandable && !expanded && code.split('\n').length > 15
    ? code.split('\n').slice(0, 15).join('\n') + '\n// ... (click to expand)'
    : code;

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h3>{title}</h3>
        {copyable && (
          <button
            className={clsx(styles.copyButton, copied && styles.copied)}
            onClick={copyToClipboard}
            title="Copy to clipboard"
          >
            {copied ? '✓ Copied!' : 'Copy'}
          </button>
        )}
      </div>
      <div className={styles.descriptionContainer}>
        <p>{description}</p>
      </div>
      <div className={styles.codeContainer}>
        <pre
          className={clsx('prism-code', styles.codeBlock)}
          style={{ maxHeight: expanded ? 'none' : maxHeight }}
        >
          <SyntaxHighlightedCode code={displayedCode} language={language} />
        </pre>
        {expandable && code.split('\n').length > 15 && (
          <button
            className={styles.expandButton}
            onClick={() => setExpanded(!expanded)}
          >
            {expanded ? 'Collapse' : 'Expand'} Code
          </button>
        )}
      </div>
    </div>
  );
}