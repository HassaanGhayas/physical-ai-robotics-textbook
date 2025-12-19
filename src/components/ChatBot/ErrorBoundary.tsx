/**
 * Error Boundary Component
 * Catches React component errors and displays fallback UI
 */

import React, { Component, ReactNode } from 'react';

interface Props {
  children: ReactNode;
}

interface State {
  hasError: boolean;
  error?: Error;
}

export default class ErrorBoundary extends Component<Props, State> {
  constructor(props: Props) {
    super(props);
    this.state = { hasError: false };
  }

  static getDerivedStateFromError(error: Error): State {
    return { hasError: true, error };
  }

  componentDidCatch(error: Error, errorInfo: React.ErrorInfo) {
    console.error('[ErrorBoundary] Caught error:', error, errorInfo);
  }

  handleReset = () => {
    this.setState({ hasError: false, error: undefined });
  };

  render() {
    if (this.state.hasError) {
      return (
        <div style={{
          padding: '2rem',
          textAlign: 'center',
          backgroundColor: 'var(--ifm-background-color)',
          borderRadius: '0.5rem',
        }}>
          <h3>Something went wrong</h3>
          <p style={{ color: 'var(--ifm-color-secondary)' }}>
            The chat interface encountered an error. Please try refreshing the page.
          </p>
          <button
            onClick={this.handleReset}
            style={{
              marginTop: '1rem',
              padding: '0.5rem 1rem',
              backgroundColor: 'var(--ifm-color-primary)',
              color: 'white',
              border: 'none',
              borderRadius: '0.25rem',
              cursor: 'pointer',
            }}
          >
            Try Again
          </button>
        </div>
      );
    }

    return this.props.children;
  }
}
