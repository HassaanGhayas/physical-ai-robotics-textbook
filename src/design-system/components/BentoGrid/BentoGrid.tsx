import React from 'react';
import styles from './BentoGrid.module.css';
import { cn } from '@/lib/utils';

export interface BentoGridItem {
  id: string;
  content: React.ReactNode;
  span?: {
    mobile?: number;
    tablet?: number;
    desktop?: number;
  };
  aspectRatio?: 'square' | 'portrait' | 'landscape' | 'wide' | 'auto';
  interactive?: boolean;
  onClick?: () => void;
}

export interface BentoGridProps {
  items: BentoGridItem[];
  columns?: {
    mobile?: number;
    tablet?: number;
    desktop?: number;
  };
  gap?: {
    mobile?: string;
    tablet?: string;
    desktop?: string;
  };
  className?: string;
  testId?: string;
}

export function BentoGrid({
  items,
  columns = { mobile: 1, tablet: 2, desktop: 3 },
  gap = { mobile: '16px', tablet: '20px', desktop: '24px' },
  className,
  testId,
}: BentoGridProps) {
  return (
    <div
      className={cn(styles.bentoGrid, className)}
      data-testid={testId}
      style={{
        '--grid-cols-mobile': columns.mobile,
        '--grid-cols-tablet': columns.tablet,
        '--grid-cols-desktop': columns.desktop,
        '--grid-gap-mobile': gap.mobile,
        '--grid-gap-tablet': gap.tablet,
        '--grid-gap-desktop': gap.desktop,
      } as React.CSSProperties}
    >
      {items.map((item) => (
        <BentoGridItem key={item.id} {...item} />
      ))}
    </div>
  );
}

interface BentoGridItemProps extends BentoGridItem {}

function BentoGridItem({
  content,
  span = { mobile: 1, tablet: 1, desktop: 1 },
  aspectRatio = 'auto',
  interactive = false,
  onClick,
}: BentoGridItemProps) {
  const itemClass = cn(
    styles.bentoGridItem,
    interactive && styles.interactive,
    styles[`aspect-${aspectRatio}`]
  );

  return (
    <div
      className={itemClass}
      style={{
        '--span-mobile': span.mobile || 1,
        '--span-tablet': span.tablet || 1,
        '--span-desktop': span.desktop || 1,
      } as React.CSSProperties}
      onClick={onClick}
      role={interactive ? 'button' : undefined}
      tabIndex={interactive ? 0 : undefined}
    >
      {content}
    </div>
  );
}
