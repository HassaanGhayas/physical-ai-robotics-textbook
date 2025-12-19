/**
 * SkipLink Component
 * Provides keyboard users with a way to skip to main content
 * Appears only when focused, adhering to WCAG 2.4.1
 */

import React from 'react';

export interface SkipLinkProps {
  href: string;
  children: React.ReactNode;
  className?: string;
}

export function SkipLink({ href, children, className }: SkipLinkProps) {
  return (
    <a
      href={href}
      className={`skip-link ${className || ''}`}
      tabIndex={0}
    >
      {children}
    </a>
  );
}

export default SkipLink;
