/**
 * LottieAnimation Component
 * Wrapper for lottie-react with accessibility and reduced motion support
 */

import React from 'react';
import Lottie, { LottieComponentProps } from 'lottie-react';
import { useReducedMotion } from '@/design-system/hooks';

export interface LottieAnimationProps extends Omit<LottieComponentProps, 'animationData'> {
  animationData: any;
  fallbackImage?: string;
  ariaLabel?: string;
  className?: string;
}

export function LottieAnimation({
  animationData,
  fallbackImage,
  ariaLabel,
  className,
  loop = true,
  autoplay = true,
  ...props
}: LottieAnimationProps) {
  const prefersReducedMotion = useReducedMotion();

  // Show static fallback if user prefers reduced motion
  if (prefersReducedMotion && fallbackImage) {
    return (
      <img
        src={fallbackImage}
        alt={ariaLabel || 'Animation'}
        className={className}
        role="img"
      />
    );
  }

  // If reduced motion but no fallback, show first frame
  if (prefersReducedMotion) {
    return (
      <Lottie
        animationData={animationData}
        loop={false}
        autoplay={false}
        className={className}
        aria-label={ariaLabel}
        role="img"
        {...props}
      />
    );
  }

  return (
    <Lottie
      animationData={animationData}
      loop={loop}
      autoplay={autoplay}
      className={className}
      aria-label={ariaLabel}
      role="img"
      {...props}
    />
  );
}
