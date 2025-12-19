/**
 * Accessibility Reporter
 * Configures axe-core for runtime accessibility testing in development
 */

import React from 'react';

let axe: any = null;

/**
 * Initialize axe-core for accessibility testing
 * Only runs in development mode
 */
export async function initA11yReporter() {
  if (process.env.NODE_ENV === 'development' && typeof window !== 'undefined') {
    try {
      // Dynamically import @axe-core/react only in development
      const axeCore = await import('@axe-core/react');
      const ReactDOM = await import('react-dom');

      axe = axeCore.default;

      // Initialize axe with React and ReactDOM
      await axe(React, ReactDOM, 1000, {
        // Configuration options
        rules: [
          {
            id: 'color-contrast',
            enabled: true,
          },
          {
            id: 'landmark-one-main',
            enabled: true,
          },
          {
            id: 'region',
            enabled: true,
          },
          {
            id: 'document-title',
            enabled: true,
          },
          {
            id: 'html-has-lang',
            enabled: true,
          },
          {
            id: 'aria-roles',
            enabled: true,
          },
          {
            id: 'aria-allowed-attr',
            enabled: true,
          },
          {
            id: 'aria-required-attr',
            enabled: true,
          },
          {
            id: 'aria-valid-attr-value',
            enabled: true,
          },
          {
            id: 'button-name',
            enabled: true,
          },
          {
            id: 'link-name',
            enabled: true,
          },
          {
            id: 'image-alt',
            enabled: true,
          },
          {
            id: 'label',
            enabled: true,
          },
        ],
      });

      console.log('✅ Accessibility testing enabled (axe-core)');
    } catch (error) {
      console.error('Failed to initialize axe-core:', error);
    }
  }
}

/**
 * Run manual accessibility audit
 * Can be called from browser console
 */
export async function runA11yAudit() {
  if (typeof window === 'undefined') {
    console.warn('A11y audit can only run in the browser');
    return;
  }

  try {
    const axeCore = await import('axe-core');
    const results = await axeCore.default.run();

    console.group('🔍 Accessibility Audit Results');
    console.log('Violations:', results.violations.length);
    console.log('Passes:', results.passes.length);
    console.log('Incomplete:', results.incomplete.length);

    if (results.violations.length > 0) {
      console.group('❌ Violations');
      results.violations.forEach((violation) => {
        console.log(`\n${violation.impact?.toUpperCase()}: ${violation.help}`);
        console.log(`Description: ${violation.description}`);
        console.log(`Help URL: ${violation.helpUrl}`);
        console.log('Affected nodes:', violation.nodes.length);
        violation.nodes.forEach((node) => {
          console.log('  -', node.html);
        });
      });
      console.groupEnd();
    }

    console.groupEnd();

    return results;
  } catch (error) {
    console.error('Failed to run accessibility audit:', error);
  }
}

// Make runA11yAudit available globally in development
if (process.env.NODE_ENV === 'development' && typeof window !== 'undefined') {
  (window as any).runA11yAudit = runA11yAudit;
}
