/**
 * FadeIn Animation Component
 * Provides fade-in animation with motion-safe defaults
 * Uses Framer Motion with LazyMotion for optimal bundle size
 */

import React from 'react';
import { LazyMotion, domAnimation, m } from 'framer-motion';
import { useReducedMotion } from '@/design-system/hooks';

export interface FadeInProps {
  children: React.ReactNode;
  duration?: number;
  delay?: number;
  direction?: 'up' | 'down' | 'left' | 'right' | 'none';
  distance?: number;
  className?: string;
  once?: boolean;
}

export function FadeIn({
  children,
  duration = 0.5,
  delay = 0,
  direction = 'up',
  distance = 20,
  className,
  once = true,
}: FadeInProps) {
  const prefersReducedMotion = useReducedMotion();

  // Disable animations if user prefers reduced motion
  if (prefersReducedMotion) {
    return <div className={className}>{children}</div>;
  }

  // Calculate initial position based on direction
  const getInitialPosition = () => {
    switch (direction) {
      case 'up':
        return { y: distance, opacity: 0 };
      case 'down':
        return { y: -distance, opacity: 0 };
      case 'left':
        return { x: distance, opacity: 0 };
      case 'right':
        return { x: -distance, opacity: 0 };
      case 'none':
      default:
        return { opacity: 0 };
    }
  };

  const getAnimatePosition = () => {
    switch (direction) {
      case 'up':
      case 'down':
        return { y: 0, opacity: 1 };
      case 'left':
      case 'right':
        return { x: 0, opacity: 1 };
      case 'none':
      default:
        return { opacity: 1 };
    }
  };

  return (
    <LazyMotion features={domAnimation}>
      <m.div
        initial={getInitialPosition()}
        whileInView={getAnimatePosition()}
        viewport={{ once, margin: '-50px' }}
        transition={{
          duration,
          delay,
          ease: [0.25, 0.1, 0.25, 1], // Custom easing
        }}
        className={className}
      >
        {children}
      </m.div>
    </LazyMotion>
  );
}
