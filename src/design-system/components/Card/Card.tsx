import React from 'react';
import styles from './Card.module.css';
import { cn } from '@/lib/utils';

export interface CardProps {
  children: React.ReactNode;
  variant?: 'default' | 'elevated' | 'outlined';
  interactive?: boolean;
  onClick?: () => void;
  padding?: 'sm' | 'md' | 'lg';
  className?: string;
  testId?: string;
}

export function Card({
  children,
  variant = 'default',
  interactive = false,
  onClick,
  padding = 'md',
  className,
  testId,
}: CardProps) {
  const cardClass = cn(
    styles.card,
    styles[variant],
    styles[`padding-${padding}`],
    interactive && styles.interactive,
    className
  );

  return (
    <div
      className={cardClass}
      onClick={onClick}
      role={interactive ? 'button' : undefined}
      tabIndex={interactive ? 0 : undefined}
      data-testid={testId}
    >
      {children}
    </div>
  );
}
