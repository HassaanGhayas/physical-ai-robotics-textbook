/**
 * Component API Contracts for UI/UX Design System
 *
 * TypeScript interfaces for all design system components.
 * These contracts define the public API that components must implement.
 */

// ============================================================================
// Base Types
// ============================================================================

export type SpacingToken = '0' | '1' | '2' | '3' | '4' | '5' | '6' | '8' | '10' | '12' | '16' | '20' | '24';
export type GrayscaleToken = '0' | '50' | '100' | '200' | '300' | '400' | '500' | '600' | '700' | '800' | '900' | '950' | '1000';
export type SemanticColorToken =
  | 'text-primary'
  | 'text-secondary'
  | 'text-tertiary'
  | 'text-disabled'
  | 'text-inverse'
  | 'bg-primary'
  | 'bg-secondary'
  | 'bg-tertiary'
  | 'bg-inverse'
  | 'border-default'
  | 'border-strong'
  | 'border-subtle';

export type BorderRadiusToken = 'none' | 'sm' | 'DEFAULT' | 'md' | 'lg' | 'xl' | 'full';
export type ShadowToken = 'none' | 'sm' | 'DEFAULT' | 'md' | 'lg' | 'xl' | 'focus';
export type AnimationDuration = 'instant' | 'fast' | 'normal' | 'slow' | 'slower' | 'slowest';
export type AnimationEasing = 'linear' | 'easeIn' | 'easeOut' | 'easeInOut' | 'bounce' | 'smooth';

// ============================================================================
// Bento Grid Component
// ============================================================================

export interface BentoGridProps {
  /** Array of items to display in the grid */
  items: BentoGridItem[];

  /** Column configuration for different breakpoints */
  columns?: {
    mobile?: number;    // Default: 1
    tablet?: number;    // Default: 2
    desktop?: number;   // Default: 3
  };

  /** Gap between grid items */
  gap?: {
    mobile?: SpacingToken;   // Default: '4' (16px)
    tablet?: SpacingToken;   // Default: '5' (20px)
    desktop?: SpacingToken;  // Default: '6' (24px)
  };

  /** Additional CSS class names */
  className?: string;

  /** Test ID for testing */
  testId?: string;
}

export interface BentoGridItem {
  /** Unique identifier for the item */
  id: string;

  /** Content to render inside the grid item */
  content: React.ReactNode;

  /** How many columns this item should span at different breakpoints */
  span?: {
    mobile?: number;   // 1 (always full width on mobile)
    tablet?: number;   // 1 or 2
    desktop?: number;  // 1, 2, or 3
  };

  /** Aspect ratio for the card */
  aspectRatio?: 'square' | 'portrait' | 'landscape' | 'wide' | 'auto';

  /** Optional click handler */
  onClick?: () => void;

  /** Whether the item is interactive (shows hover states) */
  interactive?: boolean;
}

// ============================================================================
// Animation Components
// ============================================================================

export interface FadeInProps {
  /** Content to animate */
  children: React.ReactNode;

  /** Animation duration (default: 'slow' = 300-500ms) */
  duration?: AnimationDuration;

  /** Animation delay in ms */
  delay?: number;

  /** Vertical translation amount in px (default: 16px) */
  translateY?: number;

  /** Animation easing function (default: 'easeOut') */
  easing?: AnimationEasing;

  /** Whether to respect prefers-reduced-motion (default: true) */
  respectReducedMotion?: boolean;

  /** Trigger animation when element enters viewport (default: true) */
  triggerOnView?: boolean;

  /** Viewport threshold for triggering animation (0-1, default: 0.1) */
  viewportThreshold?: number;
}

export interface LottieAnimationProps {
  /** Lottie animation data (JSON object) */
  animationData: object;

  /** Whether the animation should loop (default: false) */
  loop?: boolean;

  /** Whether the animation should autoplay (default: true) */
  autoplay?: boolean;

  /** Maximum file size in KB (enforced: 100KB) */
  maxSize?: number;

  /** Fallback static image URL for prefers-reduced-motion */
  fallbackImage?: string;

  /** Width of the animation container */
  width?: string | number;

  /** Height of the animation container */
  height?: string | number;

  /** Additional CSS class names */
  className?: string;

  /** Callback when animation completes */
  onComplete?: () => void;

  /** Test ID for testing */
  testId?: string;
}

// ============================================================================
// Accessibility Components
// ============================================================================

export interface SkipLinkProps {
  /** Target element ID to skip to */
  targetId: string;

  /** Text to display in the skip link */
  text?: string;  // Default: "Skip to main content"

  /** Additional CSS class names */
  className?: string;
}

export interface FocusTrapProps {
  /** Content to trap focus within */
  children: React.ReactNode;

  /** Whether the focus trap is active */
  active: boolean;

  /** Initial focus element selector */
  initialFocus?: string;

  /** Callback when user tries to escape */
  onEscape?: () => void;
}

// ============================================================================
// Typography Components
// ============================================================================

export interface HeadingProps {
  /** Heading level (h1-h6) */
  level: 1 | 2 | 3 | 4 | 5 | 6;

  /** Content of the heading */
  children: React.ReactNode;

  /** Optional ID for anchor links */
  id?: string;

  /** Additional CSS class names */
  className?: string;

  /** Override default font size */
  fontSize?: string;

  /** Override default font weight */
  fontWeight?: 400 | 500 | 600 | 700;
}

export interface TextProps {
  /** Content of the text */
  children: React.ReactNode;

  /** Variant of the text */
  variant?: 'body' | 'bodyLarge' | 'bodySmall' | 'caption';

  /** Color token */
  color?: SemanticColorToken;

  /** Additional CSS class names */
  className?: string;

  /** HTML element to render as */
  as?: 'p' | 'span' | 'div';
}

// ============================================================================
// Card Component
// ============================================================================

export interface CardProps {
  /** Card content */
  children: React.ReactNode;

  /** Card variant */
  variant?: 'default' | 'elevated' | 'outlined';

  /** Whether the card is interactive */
  interactive?: boolean;

  /** Click handler (makes card interactive) */
  onClick?: () => void;

  /** Padding size */
  padding?: SpacingToken;

  /** Border radius */
  borderRadius?: BorderRadiusToken;

  /** Shadow elevation */
  shadow?: ShadowToken;

  /** Additional CSS class names */
  className?: string;

  /** Test ID for testing */
  testId?: string;
}

// ============================================================================
// Button Component (shadcn/ui)
// ============================================================================

export interface ButtonProps extends React.ButtonHTMLAttributes<HTMLButtonElement> {
  /** Button variant */
  variant?: 'default' | 'outline' | 'ghost' | 'link';

  /** Button size */
  size?: 'sm' | 'md' | 'lg';

  /** Whether button is loading */
  isLoading?: boolean;

  /** Loading text to display */
  loadingText?: string;

  /** Icon to display before text */
  leftIcon?: React.ReactNode;

  /** Icon to display after text */
  rightIcon?: React.ReactNode;

  /** Additional CSS class names */
  className?: string;
}

// ============================================================================
// Theme Configuration
// ============================================================================

export interface ThemeConfig {
  /** Color system */
  colors: {
    grayscale: {
      [key in GrayscaleToken]: string;
    };
    semantic: {
      text: Record<string, GrayscaleToken>;
      background: Record<string, GrayscaleToken>;
      border: Record<string, GrayscaleToken>;
      interactive: Record<string, GrayscaleToken>;
    };
  };

  /** Spacing system */
  spacing: {
    [key in SpacingToken]: string;
  };

  /** Typography system */
  typography: {
    fontFamily: {
      base: string[];
      mono: string[];
    };
    fontSize: Record<string, string>;
    fontWeight: Record<string, number>;
    lineHeight: Record<string, number>;
  };

  /** Border radius system */
  borderRadius: {
    [key in BorderRadiusToken]: string;
  };

  /** Shadow system */
  shadows: {
    light: {
      [key in ShadowToken]: string;
    };
    dark: {
      [key in ShadowToken]: string;
    };
  };

  /** Animation system */
  animations: {
    duration: {
      [key in AnimationDuration]: string;
    };
    easing: {
      [key in AnimationEasing]: string;
    };
  };

  /** Responsive breakpoints */
  breakpoints: {
    sm: string;
    md: string;
    lg: string;
    xl: string;
    '2xl': string;
  };

  /** Maximum content width */
  maxWidth: {
    container: string;
  };
}

// ============================================================================
// Hooks
// ============================================================================

export interface UseReducedMotionReturn {
  /** Whether user prefers reduced motion */
  prefersReducedMotion: boolean;

  /** Get animation duration based on preference (0ms if reduced motion) */
  getDuration: (normalDuration: AnimationDuration) => number;
}

export interface UseColorModeReturn {
  /** Current color mode */
  colorMode: 'light' | 'dark';

  /** Function to set color mode */
  setColorMode: (mode: 'light' | 'dark') => void;

  /** Function to toggle color mode */
  toggleColorMode: () => void;
}

export interface UseBreakpointReturn {
  /** Current breakpoint */
  current: 'sm' | 'md' | 'lg' | 'xl' | '2xl';

  /** Whether current viewport is mobile */
  isMobile: boolean;

  /** Whether current viewport is tablet */
  isTablet: boolean;

  /** Whether current viewport is desktop */
  isDesktop: boolean;
}

// ============================================================================
// Utility Types
// ============================================================================

export type ResponsiveValue<T> = T | {
  mobile?: T;
  tablet?: T;
  desktop?: T;
};

export type ComponentState = 'default' | 'hover' | 'active' | 'focus' | 'disabled';

export interface AccessibilityProps {
  /** ARIA label */
  'aria-label'?: string;

  /** ARIA labelledby reference */
  'aria-labelledby'?: string;

  /** ARIA describedby reference */
  'aria-describedby'?: string;

  /** ARIA role */
  role?: string;

  /** Tab index */
  tabIndex?: number;
}
