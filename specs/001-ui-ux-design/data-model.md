# Data Model: Design Tokens & Theme Configuration

**Date**: 2025-12-18
**Feature**: UI/UX Design System Enhancement
**Purpose**: Define all design tokens, theme configuration, and design system data structures

## Overview

This document defines the complete design token schema, theme configuration structure, and component state mappings for the Physical AI & Humanoid Robotics documentation site. All values are technology-agnostic and can be implemented in Tailwind CSS, CSS custom properties, or any other styling system.

---

## 1. Color System: Grayscale Palette

### Primary Grayscale Scale (0-1000)

```javascript
// Light Mode (Default)
const grayscaleLight = {
  0: '#FFFFFF',      // Pure white - backgrounds
  50: '#FAFAFA',     // Lightest gray - elevated surfaces
  100: '#F5F5F5',    // Very light gray - subtle backgrounds
  200: '#E5E5E5',    // Light gray - borders, dividers
  300: '#D4D4D4',    // Medium-light gray - disabled text
  400: '#A3A3A3',    // Medium gray - placeholder text
  500: '#737373',    // Balanced gray - secondary text
  600: '#525252',    // Medium-dark gray - body text
  700: '#404040',    // Dark gray - primary text
  800: '#262626',    // Darker gray - headings
  900: '#171717',    // Very dark gray - emphasis
  950: '#0A0A0A',    // Near black - maximum contrast
  1000: '#000000',   // Pure black - reserved for high emphasis
};

// Dark Mode (Inverted)
const grayscaleDark = {
  0: '#0A0A0A',      // Near black - backgrounds
  50: '#171717',     // Very dark gray - elevated surfaces
  100: '#262626',    // Dark gray - subtle backgrounds
  200: '#404040',    // Medium-dark gray - borders, dividers
  300: '#525252',    // Medium gray - disabled text
  400: '#737373',    // Balanced gray - placeholder text
  500: '#A3A3A3',    // Medium-light gray - secondary text
  600: '#D4D4D4',    // Light gray - body text
  700: '#E5E5E5',    // Very light gray - primary text
  800: '#F5F5F5',    // Lightest gray - headings
  900: '#FAFAFA',    // Near white - emphasis
  950: '#FFFFFF',    // Pure white - maximum contrast
  1000: '#FFFFFF',   // Pure white - reserved for high emphasis
};
```

### Semantic Color Tokens

```javascript
// These map to grayscale values but provide semantic meaning
const semanticTokens = {
  // Text colors
  text: {
    primary: 'grayscale-700',      // Main body text
    secondary: 'grayscale-500',    // Secondary information
    tertiary: 'grayscale-400',     // Least important text
    disabled: 'grayscale-300',     // Disabled state
    inverse: 'grayscale-50',       // Text on dark backgrounds
  },

  // Background colors
  background: {
    primary: 'grayscale-0',        // Main background
    secondary: 'grayscale-50',     // Elevated surfaces
    tertiary: 'grayscale-100',     // Subtle backgrounds
    inverse: 'grayscale-950',      // Dark backgrounds
  },

  // Border colors
  border: {
    default: 'grayscale-200',      // Standard borders
    strong: 'grayscale-300',       // Emphasized borders
    subtle: 'grayscale-100',       // Minimal borders
  },

  // Interactive states
  interactive: {
    default: 'grayscale-700',      // Default interactive elements
    hover: 'grayscale-800',        // Hover state
    active: 'grayscale-900',       // Active/pressed state
    disabled: 'grayscale-300',     // Disabled state
    focus: 'grayscale-900',        // Focus state (ring color)
  },
};
```

### Contrast Validation Matrix

All text-on-background combinations must meet WCAG AA standards:

| Text Size | Background | Minimum Contrast | Validated Combinations |
|-----------|------------|------------------|----------------------|
| Body (14-18px) | grayscale-0 | 4.5:1 | text-primary (700), text-secondary (500)* |
| Body (14-18px) | grayscale-50 | 4.5:1 | text-primary (700), text-secondary (500)* |
| Large (18px+) | grayscale-0 | 3:1 | All text tokens ✓ |
| Large (18px+) | grayscale-50 | 3:1 | All text tokens ✓ |
| Body (14-18px) | grayscale-950 (dark) | 4.5:1 | text-inverse (50), text-primary (700 in dark mode = 700) ✓ |

*Requires validation - may need adjustment to grayscale-600 for 4.5:1 ratio

---

## 2. Spacing System

### Base Unit: 4px Scale

```javascript
const spacing = {
  0: '0px',       // No space
  1: '4px',       // Tightest - icon spacing
  2: '8px',       // Very tight - inline elements
  3: '12px',      // Tight - compact layouts
  4: '16px',      // Standard - most common spacing
  5: '20px',      // Medium - paragraph spacing
  6: '24px',      // Relaxed - subsection spacing
  8: '32px',      // Spacious - section spacing (min required)
  10: '40px',     // Very spacious
  12: '48px',     // Extra spacious
  16: '64px',     // Maximum - major section breaks
  20: '80px',     // Hero spacing
  24: '96px',     // Page-level spacing
};
```

### Semantic Spacing Tokens

```javascript
const semanticSpacing = {
  // Inline spacing (within components)
  inline: {
    xs: 'spacing-1',   // 4px - icon-to-text
    sm: 'spacing-2',   // 8px - button padding
    md: 'spacing-4',   // 16px - default inline space
    lg: 'spacing-6',   // 24px - larger inline space
  },

  // Stack spacing (between components)
  stack: {
    xs: 'spacing-2',   // 8px - tight vertical rhythm
    sm: 'spacing-4',   // 16px - subsection spacing
    md: 'spacing-6',   // 24px - related sections
    lg: 'spacing-8',   // 32px - major sections (required)
    xl: 'spacing-12',  // 48px - page sections
  },

  // Container padding
  container: {
    mobile: 'spacing-4',    // 16px - mobile edge padding
    tablet: 'spacing-6',    // 24px - tablet edge padding
    desktop: 'spacing-8',   // 32px - desktop edge padding
  },

  // Bento grid gaps
  grid: {
    default: 'spacing-4',   // 16px - mobile gap
    tablet: 'spacing-5',    // 20px - tablet gap
    desktop: 'spacing-6',   // 24px - desktop gap
  },
};
```

---

## 3. Typography System

### Font Family

```javascript
const fontFamily = {
  base: [
    'Inter Variable',
    'system-ui',
    '-apple-system',
    'BlinkMacSystemFont',
    'Segoe UI',
    'Roboto',
    'sans-serif'
  ],
  mono: [
    'ui-monospace',
    'SFMono-Regular',
    'Consolas',
    'Liberation Mono',
    'Menlo',
    'monospace'
  ],
};
```

### Font Size Scale

```javascript
const fontSize = {
  xs: '12px',      // 0.75rem - captions, footnotes
  sm: '14px',      // 0.875rem - small body text
  base: '16px',    // 1rem - default body text
  lg: '18px',      // 1.125rem - large body text
  xl: '20px',      // 1.25rem - small headings
  '2xl': '24px',   // 1.5rem - h4
  '3xl': '30px',   // 1.875rem - h3
  '4xl': '36px',   // 2.25rem - h2
  '5xl': '48px',   // 3rem - h1
  '6xl': '60px',   // 3.75rem - hero text
  '7xl': '72px',   // 4.5rem - display text
};
```

### Font Weight

```javascript
const fontWeight = {
  normal: 400,     // Body text
  medium: 500,     // Emphasis within body
  semibold: 600,   // Subheadings, button text
  bold: 700,       // Headings, strong emphasis
};
```

### Line Height

```javascript
const lineHeight = {
  tight: 1.25,     // Headings (h1-h6)
  snug: 1.375,     // Subheadings
  normal: 1.5,     // Body text (default)
  relaxed: 1.625,  // Comfortable reading
  loose: 2,        // Maximum spacing for accessibility
};
```

### Typography Scale Mapping

```javascript
const typographyScale = {
  display: {
    fontSize: fontSize['7xl'],    // 72px
    fontWeight: fontWeight.bold,   // 700
    lineHeight: lineHeight.tight,  // 1.25
    letterSpacing: '-0.02em',      // -2% tighter
  },
  h1: {
    fontSize: fontSize['5xl'],     // 48px
    fontWeight: fontWeight.bold,   // 700
    lineHeight: lineHeight.tight,  // 1.25
    letterSpacing: '-0.01em',      // -1% tighter
  },
  h2: {
    fontSize: fontSize['4xl'],     // 36px
    fontWeight: fontWeight.semibold, // 600
    lineHeight: lineHeight.tight,  // 1.25
  },
  h3: {
    fontSize: fontSize['3xl'],     // 30px
    fontWeight: fontWeight.semibold, // 600
    lineHeight: lineHeight.snug,   // 1.375
  },
  h4: {
    fontSize: fontSize['2xl'],     // 24px
    fontWeight: fontWeight.semibold, // 600
    lineHeight: lineHeight.snug,   // 1.375
  },
  h5: {
    fontSize: fontSize.xl,         // 20px
    fontWeight: fontWeight.semibold, // 600
    lineHeight: lineHeight.normal, // 1.5
  },
  h6: {
    fontSize: fontSize.lg,         // 18px
    fontWeight: fontWeight.semibold, // 600
    lineHeight: lineHeight.normal, // 1.5
  },
  body: {
    fontSize: fontSize.base,       // 16px
    fontWeight: fontWeight.normal, // 400
    lineHeight: lineHeight.normal, // 1.5
  },
  bodyLarge: {
    fontSize: fontSize.lg,         // 18px
    fontWeight: fontWeight.normal, // 400
    lineHeight: lineHeight.relaxed,// 1.625
  },
  bodySmall: {
    fontSize: fontSize.sm,         // 14px
    fontWeight: fontWeight.normal, // 400
    lineHeight: lineHeight.normal, // 1.5
  },
  caption: {
    fontSize: fontSize.xs,         // 12px
    fontWeight: fontWeight.normal, // 400
    lineHeight: lineHeight.normal, // 1.5
  },
};
```

---

## 4. Border Radius System

```javascript
const borderRadius = {
  none: '0px',         // Sharp corners
  sm: '4px',           // Subtle rounding - small buttons
  DEFAULT: '8px',      // Standard rounding - cards, buttons
  md: '12px',          // Medium rounding - larger cards
  lg: '16px',          // Large rounding - hero sections
  xl: '24px',          // Extra large - special elements
  full: '9999px',      // Circular - avatars, pills
};

// Semantic radius tokens
const semanticRadius = {
  card: 'borderRadius.DEFAULT',       // 8px
  cardLarge: 'borderRadius.md',       // 12px
  button: 'borderRadius.DEFAULT',     // 8px
  input: 'borderRadius.DEFAULT',      // 8px
  modal: 'borderRadius.lg',           // 16px
  pill: 'borderRadius.full',          // 9999px
};
```

---

## 5. Shadow System (Elevation)

```javascript
const boxShadow = {
  // No shadow
  none: 'none',

  // Elevation 1: Subtle lift (cards on page)
  sm: '0 2px 4px rgba(0, 0, 0, 0.04), 0 1px 2px rgba(0, 0, 0, 0.06)',

  // Elevation 2: Standard cards
  DEFAULT: '0 2px 8px rgba(0, 0, 0, 0.08), 0 1px 4px rgba(0, 0, 0, 0.06)',

  // Elevation 3: Raised cards, dropdowns
  md: '0 4px 12px rgba(0, 0, 0, 0.10), 0 2px 6px rgba(0, 0, 0, 0.08)',

  // Elevation 4: Modals, popovers
  lg: '0 8px 16px rgba(0, 0, 0, 0.12), 0 4px 8px rgba(0, 0, 0, 0.08)',

  // Elevation 5: Maximum elevation (dialogs)
  xl: '0 12px 24px rgba(0, 0, 0, 0.14), 0 8px 12px rgba(0, 0, 0, 0.10)',

  // Focus ring (accessibility)
  focus: '0 0 0 3px rgba(0, 0, 0, 0.1)',
};

// Dark mode shadows (lighter, less opacity)
const boxShadowDark = {
  sm: '0 2px 4px rgba(0, 0, 0, 0.2), 0 1px 2px rgba(0, 0, 0, 0.3)',
  DEFAULT: '0 2px 8px rgba(0, 0, 0, 0.3), 0 1px 4px rgba(0, 0, 0, 0.4)',
  md: '0 4px 12px rgba(0, 0, 0, 0.4), 0 2px 6px rgba(0, 0, 0, 0.5)',
  lg: '0 8px 16px rgba(0, 0, 0, 0.5), 0 4px 8px rgba(0, 0, 0, 0.6)',
  xl: '0 12px 24px rgba(0, 0, 0, 0.6), 0 8px 12px rgba(0, 0, 0, 0.7)',
  focus: '0 0 0 3px rgba(255, 255, 255, 0.2)',
};
```

---

## 6. Animation System

### Duration Tokens

```javascript
const transitionDuration = {
  instant: '0ms',      // No animation (prefers-reduced-motion)
  fast: '100ms',       // Micro-interactions (hover, focus)
  normal: '200ms',     // Standard transitions
  slow: '300ms',       // Fade-ins (minimum)
  slower: '400ms',     // Fade-ins (comfortable)
  slowest: '500ms',    // Fade-ins (maximum)
};
```

### Easing Functions

```javascript
const transitionTimingFunction = {
  // Standard easings
  linear: 'linear',
  easeIn: 'cubic-bezier(0.4, 0, 1, 1)',
  easeOut: 'cubic-bezier(0, 0, 0.2, 1)',       // Recommended for fade-ins
  easeInOut: 'cubic-bezier(0.4, 0, 0.2, 1)',

  // Custom easings
  bounce: 'cubic-bezier(0.68, -0.55, 0.265, 1.55)',
  smooth: 'cubic-bezier(0.25, 0.1, 0.25, 1)',
};
```

### Animation Presets

```javascript
const animations = {
  fadeIn: {
    duration: transitionDuration.slow,      // 300-500ms
    easing: transitionTimingFunction.easeOut,
    transform: {
      from: { opacity: 0, translateY: '16px' },
      to: { opacity: 1, translateY: '0px' },
    },
    respectReducedMotion: true,
  },

  scaleUp: {
    duration: transitionDuration.normal,    // 200ms
    easing: transitionTimingFunction.easeOut,
    transform: {
      from: { scale: 0.98 },
      to: { scale: 1 },
    },
    respectReducedMotion: true,
  },

  slideIn: {
    duration: transitionDuration.slow,      // 300ms
    easing: transitionTimingFunction.easeOut,
    transform: {
      from: { translateX: '-100%' },
      to: { translateX: '0%' },
    },
    respectReducedMotion: true,
  },
};
```

### Reduced Motion Fallback

```javascript
const reducedMotionFallback = {
  fadeIn: {
    duration: transitionDuration.fast,      // 100ms
    easing: transitionTimingFunction.linear,
    transform: {
      from: { opacity: 0 },
      to: { opacity: 1 },
    },
  },
  // All other animations: opacity only, 100ms
};
```

---

## 7. Responsive Breakpoints

```javascript
const breakpoints = {
  sm: '640px',     // Small tablets
  md: '768px',     // Tablets (bento grid: 1 → 2 columns)
  lg: '1024px',    // Small desktops
  xl: '1200px',    // Desktops (bento grid: 2 → 3 columns)
  '2xl': '1440px', // Large desktops
};

// Max width constraint
const maxWidth = {
  container: '1200px',   // Main content area max width (required)
};
```

---

## 8. Component State Mappings

### Button States

```javascript
const buttonStates = {
  default: {
    backgroundColor: 'grayscale-700',
    color: 'grayscale-0',
    border: 'none',
    shadow: boxShadow.sm,
  },
  hover: {
    backgroundColor: 'grayscale-800',
    shadow: boxShadow.DEFAULT,
    transform: 'scale(1.02)',
    transition: transitionDuration.normal,
  },
  active: {
    backgroundColor: 'grayscale-900',
    shadow: boxShadow.none,
    transform: 'scale(0.98)',
  },
  focus: {
    outline: '2px solid grayscale-900',
    outlineOffset: '2px',
    shadow: boxShadow.focus,
  },
  disabled: {
    backgroundColor: 'grayscale-300',
    color: 'grayscale-500',
    cursor: 'not-allowed',
    shadow: boxShadow.none,
  },
};
```

### Card States

```javascript
const cardStates = {
  default: {
    backgroundColor: 'grayscale-0',
    border: '1px solid grayscale-200',
    borderRadius: semanticRadius.card,
    shadow: boxShadow.sm,
  },
  hover: {
    border: '1px solid grayscale-300',
    shadow: boxShadow.DEFAULT,
    transform: 'translateY(-2px)',
    transition: transitionDuration.normal,
  },
  active: {
    border: '1px solid grayscale-400',
    shadow: boxShadow.md,
  },
  focus: {
    outline: '2px solid grayscale-700',
    outlineOffset: '2px',
  },
};
```

### Input States

```javascript
const inputStates = {
  default: {
    backgroundColor: 'grayscale-0',
    border: '1px solid grayscale-300',
    color: 'text-primary',
  },
  hover: {
    border: '1px solid grayscale-400',
  },
  focus: {
    border: '1px solid grayscale-700',
    outline: '2px solid grayscale-700',
    outlineOffset: '2px',
    shadow: boxShadow.focus,
  },
  error: {
    border: '1px solid #DC2626', // Exception: red for errors
    color: '#DC2626',
  },
  disabled: {
    backgroundColor: 'grayscale-100',
    border: '1px solid grayscale-200',
    color: 'grayscale-400',
    cursor: 'not-allowed',
  },
};
```

---

## 9. Bento Grid Configuration

```javascript
const bentoGridConfig = {
  columns: {
    mobile: 1,       // 0-767px: single column
    tablet: 2,       // 768-1199px: 2 columns
    desktop: 3,      // 1200px+: 3 columns
  },

  gaps: {
    mobile: spacing[4],     // 16px
    tablet: spacing[5],     // 20px
    desktop: spacing[6],    // 24px
  },

  // Card span options (how many columns a card occupies)
  cardSpans: {
    mobile: [1],            // Always full width
    tablet: [1, 2],         // 1 or 2 columns
    desktop: [1, 2, 3],     // 1, 2, or 3 columns
  },

  // Aspect ratios for cards
  aspectRatios: {
    square: '1 / 1',
    portrait: '3 / 4',
    landscape: '16 / 9',
    wide: '21 / 9',
    auto: 'auto',           // Content-driven height
  },
};
```

---

## 10. Theme Configuration Structure

```typescript
interface ThemeConfig {
  colors: {
    grayscale: GrayscaleColorPalette;
    semantic: SemanticColorTokens;
  };

  spacing: SpacingScale;

  typography: {
    fontFamily: FontFamilyScale;
    fontSize: FontSizeScale;
    fontWeight: FontWeightScale;
    lineHeight: LineHeightScale;
    scale: TypographyScale;
  };

  borderRadius: BorderRadiusScale;

  shadows: {
    light: ShadowScale;
    dark: ShadowScale;
  };

  animations: {
    duration: DurationTokens;
    easing: EasingFunctions;
    presets: AnimationPresets;
  };

  breakpoints: BreakpointScale;

  componentStates: {
    button: ComponentStateMapping;
    card: ComponentStateMapping;
    input: ComponentStateMapping;
  };

  bentoGrid: BentoGridConfig;
}

// Example instance
const theme: ThemeConfig = {
  colors: {
    grayscale: grayscaleLight,  // or grayscaleDark for dark mode
    semantic: semanticTokens,
  },
  spacing: spacing,
  typography: {
    fontFamily: fontFamily,
    fontSize: fontSize,
    fontWeight: fontWeight,
    lineHeight: lineHeight,
    scale: typographyScale,
  },
  borderRadius: borderRadius,
  shadows: {
    light: boxShadow,
    dark: boxShadowDark,
  },
  animations: {
    duration: transitionDuration,
    easing: transitionTimingFunction,
    presets: animations,
  },
  breakpoints: breakpoints,
  componentStates: {
    button: buttonStates,
    card: cardStates,
    input: inputStates,
  },
  bentoGrid: bentoGridConfig,
};
```

---

## 11. Accessibility Requirements

### Focus Indicators

All interactive elements MUST have visible focus indicators:

- **Minimum thickness**: 2px outline
- **Minimum contrast**: 3:1 against background
- **Recommended**: `outline: 2px solid grayscale-900; outline-offset: 2px;`

### Color Contrast Validation

All text-background combinations MUST meet:

- **Body text (14-18px)**: 4.5:1 contrast ratio
- **Large text (18px+ or bold 14px+)**: 3:1 contrast ratio
- **Focus indicators**: 3:1 contrast ratio

### Animation Accessibility

- **Respect prefers-reduced-motion**: All animations <100ms or disabled
- **No flickering**: No animations faster than 3 flashes per second
- **Pause controls**: Provide pause for auto-play animations >5 seconds

---

## 12. Dark Mode Mappings

All tokens automatically adapt to dark mode via CSS custom properties:

```css
:root {
  /* Light mode values */
}

[data-theme='dark'] {
  /* Dark mode values (inverted grayscale) */
}
```

Semantic tokens remain consistent (e.g., `text-primary` always points to the appropriate grayscale value for the current theme).

---

## Usage Example

```tsx
import { theme } from '@/design-system/tokens';

function ExampleComponent() {
  return (
    <div
      style={{
        padding: theme.spacing[6],             // 24px
        backgroundColor: theme.colors.semantic.background.primary,
        borderRadius: theme.borderRadius.DEFAULT,
        boxShadow: theme.shadows.light.DEFAULT,
      }}
    >
      <h2 style={theme.typography.scale.h2}>
        Heading with proper typography scale
      </h2>
    </div>
  );
}
```

---

## Next Steps

1. Generate TypeScript types from this data model (`contracts/component-api.ts`)
2. Export design tokens as JSON (`contracts/design-tokens.json`)
3. Create theme validation schema (`contracts/theme-schema.json`)
4. Document usage patterns in `quickstart.md`
