# Technology Research & Decisions: UI/UX Design System Enhancement

**Date**: 2025-12-18
**Feature**: UI/UX Design System
**Research Agent ID**: a92bcf7

## Executive Summary

All 10 technology decisions have been researched and documented. Total production bundle impact: **~126.6KB gzipped** (well within the 200KB budget). All recommended technologies are compatible with Docusaurus 3.9.2, React 19, and Node.js 20+.

**Priority for Implementation**:
1. **Week 1**: Tailwind CSS + shadcn/ui + Inter font (foundation)
2. **Week 2**: Framer Motion + Dark mode + SVG textures (UX enhancement)
3. **Week 3**: Accessibility testing + Performance monitoring (quality gates)
4. **Week 4**: Vitest component testing (developer velocity)

---

## Decision 1: Tailwind CSS Version

### Decision: **Tailwind CSS v3.4** (Stable)

### Rationale:
- Production-ready with extensive ecosystem support
- Well-documented integration patterns with Docusaurus
- Works with all modern and legacy browsers if needed
- Proven PostCSS integration
- Stable API with no breaking changes expected

### Alternatives Considered:
- **Tailwind v4 (Alpha)**:
  - More modern with CSS-based configuration
  - Requires Safari 16.4+, Chrome 111+, Firefox 128+
  - More complex Docusaurus integration requiring custom plugins
  - Still maturing with potential breaking changes
  - Community feedback: "downgrading from v4 to v3 for stability"

### Tradeoffs:
- v3: Mature but uses JavaScript config (more traditional approach)
- v4: Modern CSS config but bleeding edge, limited Docusaurus examples

### Bundle Impact: **~20KB gzipped**

### Installation Steps:

```bash
npm install -D tailwindcss@^3.4 postcss autoprefixer
npx tailwindcss init
```

Create `D:\my-web\plugins\tailwind-config.cjs`:
```javascript
module.exports = function () {
  return {
    name: 'tailwind-plugin',
    configurePostCss(postcssOptions) {
      postcssOptions.plugins.push(
        require('tailwindcss'),
        require('autoprefixer'),
      );
      return postcssOptions;
    },
  };
};
```

Update `D:\my-web\docusaurus.config.ts`:
```typescript
plugins: ['./plugins/tailwind-config.cjs'],
```

Update `D:\my-web\src\css\custom.css`:
```css
@tailwind base;
@tailwind components;
@tailwind utilities;
```

---

## Decision 2: shadcn/ui Integration

### Decision: **shadcn/ui with Custom Setup**

### Rationale:
- All components updated for React 19 (removed forwardRefs)
- Copy-paste architecture - you own the code, no external dependencies
- Pairs perfectly with Tailwind CSS (required dependency)
- Fully compatible with Docusaurus SSR/SSG
- Highly customizable with complete control over styling

### Alternatives Considered:
- **Building components from scratch**: Time-intensive, reinventing the wheel
- **Material-UI**: Adds 300KB+ bundle size, opinionated styling
- **Chakra UI**: Has SSR complexities, larger bundle

### Tradeoffs:
- Requires Tailwind CSS (already decided)
- Needs custom path aliases for Docusaurus
- More initial setup than traditional component libraries
- No version updates (you manually copy new components)

### Bundle Impact: **0KB** (you copy source code into your project)

### Installation Steps:

1. Install dependencies:
```bash
npm install -D @types/node
npm install class-variance-authority clsx tailwind-merge lucide-react
```

2. Create alias plugin `D:\my-web\plugins\alias-plugin.cjs`:
```javascript
const path = require('path');

module.exports = function () {
  return {
    name: 'alias-plugin',
    configureWebpack() {
      return {
        resolve: {
          alias: {
            '@': path.resolve(__dirname, '../src'),
          },
        },
      };
    },
  };
};
```

3. Update `D:\my-web\docusaurus.config.ts`:
```typescript
plugins: [
  './plugins/tailwind-config.cjs',
  './plugins/alias-plugin.cjs',
],
```

4. Create `D:\my-web\src\lib\utils.ts`:
```typescript
import { type ClassValue, clsx } from "clsx"
import { twMerge } from "tailwind-merge"

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs))
}
```

5. Initialize shadcn/ui:
```bash
npx shadcn@latest init
```

Select options:
- TypeScript: Yes
- Custom path: `@/components`
- CSS variables: Yes
- Tailwind config: `tailwind.config.js`

---

## Decision 3: Animation Library

### Decision: **Framer Motion v11 with LazyMotion**

### Rationale:
- **Bundle Optimization**: LazyMotion reduces bundle from 34KB to 4.6KB (86% reduction)
- **SSR Compatible**: First-class Docusaurus SSR support
- **60fps Performance**: Hardware-accelerated transforms and GPU acceleration
- **Reduced Motion**: Built-in `prefers-reduced-motion` support
- **Developer Experience**: Intuitive declarative API, excellent documentation

### Alternatives Considered:
- **React Spring**:
  - Better for physics-based animations
  - 30% smaller base bundle (25KB vs 34KB)
  - Steeper learning curve, less intuitive API
- **Pure CSS**:
  - Smallest bundle (0KB JavaScript)
  - Limited to simpler animations, no dynamic animations
  - Harder to coordinate complex sequences

### Tradeoffs:
- Framer Motion: Slightly larger bundle without LazyMotion, but excellent DX
- React Spring: Smaller bundle, but more complex API for simple animations
- CSS: Zero JS cost, but very limited capabilities for fade-ins and Lottie coordination

### Bundle Impact: **4.6KB gzipped** (with LazyMotion)

### Installation Steps:

```bash
npm install framer-motion@^11
```

Example with LazyMotion:
```typescript
// D:\my-web\src\components\AnimatedComponent.tsx
import { LazyMotion, domAnimation, m } from "framer-motion"

export function AnimatedComponent() {
  return (
    <LazyMotion features={domAnimation}>
      <m.div
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.5, ease: "easeOut" }}
      >
        Content
      </m.div>
    </LazyMotion>
  )
}
```

### Accessibility Implementation:
```typescript
const prefersReducedMotion = window.matchMedia(
  "(prefers-reduced-motion: reduce)"
).matches

const variants = {
  hidden: {
    opacity: 0,
    y: prefersReducedMotion ? 0 : 20
  },
  visible: {
    opacity: 1,
    y: 0,
    transition: {
      duration: prefersReducedMotion ? 0.1 : 0.5,
      ease: "easeOut"
    }
  }
}
```

---

## Decision 4: Lottie Library

### Decision: **lottie-react**

### Rationale:
- Native React hooks and component API (better DX)
- Same bundle size as lottie-web (~82KB)
- Declarative API (vs imperative lottie-web)
- Easier to implement lazy loading and code splitting
- Better TypeScript support

### Alternatives Considered:
- **lottie-web**: Same engine, but imperative API requires more boilerplate code
- **react-lottie-player**: "Lightweight" wrapper but less maintained, similar bundle size

### Tradeoffs:
- Both libraries use the same lottie-web engine, so bundle size is identical
- lottie-react provides minimal overhead (~1KB) for significantly better DX
- Imperative API (lottie-web) requires manual state management vs declarative (lottie-react)

### Bundle Impact: **~82KB + animation JSON files** (capped at 100KB per file)

### Installation Steps:

```bash
npm install lottie-react
```

Example with lazy loading:
```typescript
// D:\my-web\src\components\LottieAnimation.tsx
import { lazy, Suspense, useState, useEffect } from 'react'

const Lottie = lazy(() => import('lottie-react'))

export function LottieAnimation() {
  const [animationData, setAnimationData] = useState(null)

  useEffect(() => {
    import('../assets/animations/robot.json').then(setAnimationData)
  }, [])

  if (!animationData) return <div>Loading...</div>

  return (
    <Suspense fallback={<div>Loading animation...</div>}>
      <Lottie
        animationData={animationData}
        loop={false}
        autoplay={true}
      />
    </Suspense>
  )
}
```

### Bundle Optimization:
- Keep animation JSON files under 100KB (requirement)
- Use [LottieFiles Editor](https://lottiefiles.com/tools/lottie-editor) to optimize animations
- Lazy load animations below the fold
- Provide fallback static images for `prefers-reduced-motion` users

---

## Decision 5: Font Loading Strategy

### Decision: **Self-hosted Inter via @fontsource/inter**

### Rationale:
- **GDPR Compliant**: No data sent to Google (EU legal requirement since 2022 court ruling)
- **Performance**: With HTTP/2, self-hosting is faster (no extra DNS lookups, single connection)
- **PageSpeed**: Improves mobile scores by 20-40 points vs Google Fonts CDN
- **Privacy**: No IP address tracking or third-party cookies
- **Control**: Bundle and minify font CSS, subset characters for specific languages

### Alternatives Considered:
- **Google Fonts CDN**:
  - Violates GDPR (German court ruling 2022)
  - No browser cache sharing anymore (per-origin caching since 2020)
  - Automatic subsetting (can replicate with fontsource)
  - One less thing to self-host

### Tradeoffs:
- Self-hosted requires manual subsetting (but fontsource handles this automatically)
- Slightly larger initial bundle if not subsetted properly
- Google Fonts auto-updates (but version locking provides better stability)
- Need to manually update font versions (vs automatic CDN updates)

### Bundle Impact: **~20KB gzipped** (Variable font subset)

### Installation Steps:

```bash
npm install @fontsource-variable/inter
```

Update `D:\my-web\src\css\custom.css`:
```css
/* Import Inter Variable font */
@import '@fontsource-variable/inter';

:root {
  --ifm-font-family-base: 'Inter Variable', system-ui, -apple-system,
                          BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;

  /* Enable OpenType features for better typography */
  font-feature-settings: 'cv11', 'ss01';
  font-variation-settings: 'opsz' auto;
}

/* Optimize font loading */
@supports (font-variation-settings: normal) {
  :root {
    font-family: 'Inter Variable', sans-serif;
  }
}
```

### Advanced Subsetting (if needed):
```bash
# If you need a smaller subset for specific languages
npm install -g @fontsource/cli
fontsource subset inter --chars="ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789" --output=./static/fonts
```

---

## Decision 6: Texture Pattern Implementation

### Decision: **SVG Patterns (Data URI Inline)**

### Rationale:
- **Bundle Size**: 2.5KB for complex pattern vs 180KB for equivalent PNG
- **Scalability**: Perfect clarity on all screen resolutions (vector format)
- **GZip Compression**: 92.9% reduction when compressed by build system
- **CSS Variables**: Can theme patterns with CSS custom properties for dark mode
- **Zero HTTP Requests**: Inlined as data URI = no network cost, no CORS issues

### Alternatives Considered:
- **PNG Images**:
  - Better for photo-realistic textures (10KB-100KB)
  - Requires HTTP request or large base64 encoding
  - Not scalable (blurry on high-DPI displays)
- **CSS Gradients**:
  - Smallest for simple patterns (0KB JavaScript)
  - Very limited pattern complexity
  - Good for simple noise or gradients only

### Tradeoffs:
- SVG: Slightly more CPU for complex patterns (negligible on modern devices)
- PNG: Larger file size, extra HTTP request unless inlined as base64
- CSS: Fastest render, but extremely limited pattern capabilities

### Bundle Impact: **2-5KB gzipped**

### Implementation:

Create `D:\my-web\src\css\patterns.css`:
```css
:root {
  /* Dot pattern for light mode */
  --pattern-dots: url("data:image/svg+xml,%3Csvg width='20' height='20' xmlns='http://www.w3.org/2000/svg'%3E%3Ccircle cx='10' cy='10' r='2' fill='%23000' opacity='0.05'/%3E%3C/svg%3E");

  /* Grid pattern */
  --pattern-grid: url("data:image/svg+xml,%3Csvg width='40' height='40' xmlns='http://www.w3.org/2000/svg'%3E%3Cpath d='M0 0L40 0 40 40 0 40z' fill='none' stroke='%23000' stroke-width='1' opacity='0.03'/%3E%3C/svg%3E");

  /* Diagonal lines */
  --pattern-diagonal: url("data:image/svg+xml,%3Csvg width='30' height='30' xmlns='http://www.w3.org/2000/svg'%3E%3Cpath d='M0 0L30 30M30 0L0 30' stroke='%23000' stroke-width='1' opacity='0.03'/%3E%3C/svg%3E");
}

/* Dark mode variants */
[data-theme='dark'] {
  --pattern-dots: url("data:image/svg+xml,%3Csvg width='20' height='20' xmlns='http://www.w3.org/2000/svg'%3E%3Ccircle cx='10' cy='10' r='2' fill='%23fff' opacity='0.05'/%3E%3C/svg%3E");

  --pattern-grid: url("data:image/svg+xml,%3Csvg width='40' height='40' xmlns='http://www.w3.org/2000/svg'%3E%3Cpath d='M0 0L40 0 40 40 0 40z' fill='none' stroke='%23fff' stroke-width='1' opacity='0.03'/%3E%3C/svg%3E");
}

/* Apply background pattern */
.background-pattern {
  background-image: var(--pattern-dots);
  background-repeat: repeat;
  background-size: 20px 20px;
  background-attachment: scroll; /* Better performance than 'fixed' */
}

/* Performance optimization: reduce opacity on low-end devices */
@media (prefers-reduced-motion: reduce) {
  .background-pattern {
    background-image: none; /* Disable for accessibility */
  }
}
```

### Pattern Generation Tools:
- [Hero Patterns](https://heropatterns.com/) - SVG pattern generator with customization
- [SVGBackgrounds.com](https://www.svgbackgrounds.com/) - Customizable backgrounds
- [SVGOMG](https://jakearchibald.github.io/svgomg/) - SVG optimizer (reduce file size)

---

## Decision 7: Accessibility Testing

### Decision: **Multi-Tool Approach: axe-core + Lighthouse CI**

### Rationale:
- **axe-core**: Industry standard (Deque Systems), zero false positives, catches 30-40% of WCAG issues
- **Lighthouse CI**: Already in Chrome DevTools, tracks scores over time, provides holistic scoring
- **Complementary**: axe-core finds specific violations, Lighthouse provides overall accessibility score
- **CI Integration**: Both work seamlessly in GitHub Actions
- **Free**: No cost for unlimited runs

### Alternatives Considered:
- **Pa11y**: Simpler CLI tool, but uses axe-core under the hood (no added value over axe-core directly)
- **Only Lighthouse**: Misses some specific issues that axe-core catches (like missing form labels)
- **Manual Testing Only**: Essential but not scalable, cannot prevent regressions

### Tradeoffs:
- Automated tools catch only 30-40% of accessibility issues (manual testing still required)
- Multiple tools increase CI time by ~2 minutes (acceptable tradeoff for quality)
- Must supplement with manual keyboard navigation and screen reader testing

### Bundle Impact: **0KB** (development and CI tools only)

### Installation Steps:

#### 1. Install Dependencies:
```bash
npm install -D @axe-core/react @lhci/cli
```

#### 2. Setup axe-core for Development:
Create `D:\my-web\src\theme\Root.tsx`:
```typescript
import React, { useEffect } from 'react';

export default function Root({children}) {
  useEffect(() => {
    if (process.env.NODE_ENV !== 'production') {
      import('@axe-core/react').then(axe => {
        axe.default(React, React.ReactDOM, 1000);
      });
    }
  }, []);

  return <>{children}</>;
}
```

#### 3. Setup Lighthouse CI:
Create `D:\my-web\lighthouserc.json`:
```json
{
  "ci": {
    "collect": {
      "startServerCommand": "npm run serve",
      "url": ["http://localhost:3000"]
    },
    "assert": {
      "assertions": {
        "categories:accessibility": ["error", {"minScore": 0.9}],
        "categories:performance": ["warn", {"minScore": 0.8}],
        "categories:seo": ["warn", {"minScore": 0.9}]
      }
    }
  }
}
```

#### 4. WCAG AA Compliance Checklist:
```markdown
- [ ] Color contrast ratio ≥ 4.5:1 (normal text 14-18px)
- [ ] Color contrast ratio ≥ 3:1 (large text 18px+ or bold 14px+)
- [ ] All images have descriptive alt text
- [ ] Form inputs have associated labels
- [ ] All interactive elements keyboard accessible (Tab, Enter, Space, Arrows)
- [ ] Focus indicators visible (2px outline, 3:1 contrast)
- [ ] Heading hierarchy logical (h1 → h2 → h3, no skipping levels)
- [ ] ARIA labels where needed (buttons, landmarks, live regions)
- [ ] prefers-reduced-motion respected (<100ms animations or disabled)
- [ ] Screen reader tested (NVDA on Windows, VoiceOver on macOS)
```

---

## Decision 8: Dark Mode Implementation

### Decision: **Docusaurus Native Dark Mode with Custom Grayscale Theme**

### Rationale:
- **Built-in**: Docusaurus has robust dark mode out of the box with theme switcher
- **System Preference**: Respects `prefers-color-scheme` media query automatically
- **Persistent**: User preference saved to localStorage and synced across pages
- **React Hook**: `useColorMode()` for component-level theme detection
- **Already Configured**: Your `docusaurus.config.ts` already has `respectPrefersColorScheme: true`

### Current Configuration:
Your existing `D:\my-web\docusaurus.config.ts` already has:
```typescript
colorMode: {
  respectPrefersColorScheme: true,
}
```
This is perfect - it syncs with OS dark mode settings.

### Implementation Strategy:
Update `D:\my-web\src\css\custom.css` with grayscale theme:
```css
/* Grayscale Light Theme */
:root {
  --ifm-color-primary: #404040;
  --ifm-color-primary-dark: #2d2d2d;
  --ifm-color-primary-darker: #262626;
  --ifm-color-primary-darkest: #1a1a1a;
  --ifm-color-primary-light: #595959;
  --ifm-color-primary-lighter: #737373;
  --ifm-color-primary-lightest: #8c8c8c;

  --ifm-background-color: #ffffff;
  --ifm-background-surface-color: #f5f5f5;
  --ifm-font-color-base: #1a1a1a;
  --ifm-color-emphasis-100: #f0f0f0;
  --ifm-color-emphasis-200: #e0e0e0;
  --ifm-color-emphasis-300: #d0d0d0;
}

/* Grayscale Dark Theme */
[data-theme='dark'] {
  --ifm-color-primary: #d4d4d4;
  --ifm-color-primary-dark: #b8b8b8;
  --ifm-color-primary-darker: #a8a8a8;
  --ifm-color-primary-darkest: #8c8c8c;
  --ifm-color-primary-light: #e0e0e0;
  --ifm-color-primary-lighter: #f0f0f0;
  --ifm-color-primary-lightest: #fafafa;

  --ifm-background-color: #0a0a0a;
  --ifm-background-surface-color: #171717;
  --ifm-font-color-base: #e5e5e5;
  --ifm-color-emphasis-100: #262626;
  --ifm-color-emphasis-200: #404040;
  --ifm-color-emphasis-300: #525252;
}
```

### Component-Level Dark Mode Detection:
```typescript
import { useColorMode } from '@docusaurus/theme-common';

export function ThemedComponent() {
  const { colorMode, setColorMode } = useColorMode();

  return (
    <div className={colorMode === 'dark' ? 'dark-variant' : 'light-variant'}>
      Content adapts to theme
    </div>
  );
}
```

### Alternatives Considered:
- **Manual CSS Variables**: More work, less integration with Docusaurus theme switcher
- **Third-party libraries** (next-themes): Unnecessary, Docusaurus handles it

### Tradeoffs:
- Grayscale palette sacrifices color-coded information (must use icons/patterns to supplement)
- Must ensure all grayscale combinations maintain 4.5:1 / 3:1 contrast ratios
- May need subtle accent color for critical CTAs (carefully consider brand guidelines)

### Bundle Impact: **0KB** (built-in Docusaurus feature)

---

## Decision 9: Performance Monitoring

### Decision: **Lighthouse CI + bundlesize + GitHub Actions**

### Rationale:
- **Lighthouse CI**: Tracks performance scores over time, blocks PRs if scores drop below threshold
- **bundlesize**: Prevents bundle bloat, fails CI if bundle size limits exceeded
- **GitHub Actions**: Free for public repositories, seamless integration with GitHub
- **Trend Tracking**: See performance and bundle size trends over time
- **Automated Enforcement**: No manual intervention needed, catches regressions automatically

### Alternatives Considered:
- **Webpack Performance Hints**: Good for local dev, but doesn't block PRs automatically
- **Bundle Analyzer**: Manual inspection tool (good supplement, not replacement for automated monitoring)
- **Paid Services** (SpeedCurve, Calibre): Overkill for documentation site, expensive

### Tradeoffs:
- Adds ~3-5 minutes to CI build time (acceptable for quality assurance)
- Requires GitHub Actions runners (free for public repos, uses GitHub quotas)
- Must maintain performance budgets (requires occasional updates as site grows)

### Bundle Impact: **0KB** (CI tools only, not included in production bundle)

### Installation Steps:

#### 1. Install bundlesize:
```bash
npm install -D bundlesize
```

#### 2. Configure in `package.json`:
```json
{
  "scripts": {
    "test:bundle": "bundlesize"
  },
  "bundlesize": [
    {
      "path": "./build/assets/js/*.js",
      "maxSize": "250 kB"
    },
    {
      "path": "./build/assets/css/*.css",
      "maxSize": "50 kB"
    }
  ]
}
```

#### 3. Create Performance Budget:
Create `D:\my-web\budget.json`:
```json
[
  {
    "path": "/*",
    "timings": [
      {
        "metric": "first-contentful-paint",
        "budget": 1500
      },
      {
        "metric": "largest-contentful-paint",
        "budget": 2500
      },
      {
        "metric": "total-blocking-time",
        "budget": 300
      },
      {
        "metric": "cumulative-layout-shift",
        "budget": 0.1
      }
    ],
    "resourceSizes": [
      {
        "resourceType": "script",
        "budget": 300
      },
      {
        "resourceType": "stylesheet",
        "budget": 50
      },
      {
        "resourceType": "document",
        "budget": 50
      },
      {
        "resourceType": "total",
        "budget": 500
      }
    ]
  }
]
```

#### 4. Update `lighthouserc.json`:
```json
{
  "ci": {
    "collect": {
      "startServerCommand": "npm run serve",
      "url": ["http://localhost:3000"],
      "numberOfRuns": 3
    },
    "assert": {
      "budgetFile": "./budget.json",
      "assertions": {
        "categories:performance": ["error", {"minScore": 0.9}],
        "categories:accessibility": ["error", {"minScore": 0.9}],
        "first-contentful-paint": ["error", {"maxNumericValue": 1500}],
        "largest-contentful-paint": ["error", {"maxNumericValue": 2500}],
        "total-blocking-time": ["error", {"maxNumericValue": 300}],
        "cumulative-layout-shift": ["error", {"maxNumericValue": 0.1}]
      }
    },
    "upload": {
      "target": "temporary-public-storage"
    }
  }
}
```

---

## Decision 10: Component Testing

### Decision: **Vitest + React Testing Library**

### Rationale:
- **Speed**: 30-70% faster than Jest (cold runs 4x faster, hot reloads near-instant)
- **ES Modules**: Native ESM support (Jest requires complex configuration)
- **React 19 Ready**: Full React 19 support out of the box, no compatibility issues
- **Modern Stack**: Future-proof choice for 2025+, growing ecosystem
- **Jest Compatible**: Migrate existing Jest tests with minimal changes (similar API)
- **Watch Mode**: Superior watch mode with HMR-like instant feedback

### Alternatives Considered:
- **Jest**:
  - Mature with extensive StackOverflow answers
  - Slower (especially with TypeScript + ESM)
  - Requires more configuration for modern setups
  - Still solid choice, but aging
- **Cypress Component Testing**:
  - Heavier (runs in real browser)
  - Better for E2E tests than unit tests
  - Slower feedback loop

### Tradeoffs:
- Vitest: Newer (fewer StackOverflow answers, but great docs), rapidly growing community
- Jest: More established (abundant resources), but slower and requires more config
- Learning curve minimal if familiar with Jest (API is nearly identical)

### Bundle Impact: **0KB** (development tool only)

### Installation Steps:

#### 1. Install Dependencies:
```bash
npm install -D vitest @vitest/ui jsdom @testing-library/react @testing-library/jest-dom @testing-library/user-event
```

#### 2. Create Vitest Config:
Create `D:\my-web\vitest.config.ts`:
```typescript
import { defineConfig } from 'vitest/config'
import react from '@vitejs/plugin-react'
import path from 'path'

export default defineConfig({
  plugins: [react()],
  test: {
    globals: true,
    environment: 'jsdom',
    setupFiles: './src/test/setup.ts',
    css: true,
    coverage: {
      provider: 'v8',
      reporter: ['text', 'json', 'html'],
      exclude: [
        'node_modules/',
        'src/test/',
        '**/*.config.ts',
        '**/*.d.ts',
      ],
    },
  },
  resolve: {
    alias: {
      '@': path.resolve(__dirname, './src'),
    },
  },
})
```

#### 3. Create Test Setup:
Create `D:\my-web\src\test\setup.ts`:
```typescript
import '@testing-library/jest-dom'
import { expect, afterEach, vi } from 'vitest'
import { cleanup } from '@testing-library/react'
import * as matchers from '@testing-library/jest-dom/matchers'

expect.extend(matchers)

// Cleanup after each test
afterEach(() => {
  cleanup()
})

// Mock Docusaurus modules
vi.mock('@docusaurus/useDocusaurusContext', () => ({
  default: () => ({
    siteConfig: {
      title: 'Physical AI & Humanoid Robotics',
      url: 'https://test.com',
    },
  }),
}))

vi.mock('@docusaurus/theme-common', () => ({
  useColorMode: () => ({
    colorMode: 'light',
    setColorMode: vi.fn()
  }),
}))
```

#### 4. Update `package.json`:
```json
{
  "scripts": {
    "test": "vitest",
    "test:ui": "vitest --ui",
    "test:coverage": "vitest run --coverage"
  }
}
```

---

## Summary Matrix

| Decision | Recommendation | Primary Reason | Bundle Impact |
|----------|---------------|----------------|---------------|
| **CSS Framework** | Tailwind v3.4 | Stability + Docusaurus integration | ~20KB gzipped |
| **Component Library** | shadcn/ui | React 19 support + code ownership | 0KB (you copy code) |
| **Animation** | Framer Motion + LazyMotion | SSR + reduced bundle (4.6KB) | 4.6KB gzipped |
| **Lottie** | lottie-react | React DX + same bundle size as lottie-web | ~82KB + JSON (max 100KB per file) |
| **Fonts** | @fontsource/inter | GDPR + performance + privacy | ~20KB gzipped (variable font) |
| **Textures** | SVG patterns (inline) | 92% smaller than PNG + scalable | 2-5KB gzipped |
| **A11y Testing** | axe-core + Lighthouse CI | Industry standard + complementary | 0KB (dev/CI only) |
| **Dark Mode** | Docusaurus native | Built-in + persistent + system sync | 0KB (built-in feature) |
| **Performance** | Lighthouse CI + bundlesize | Free + GitHub Actions integration | 0KB (CI only) |
| **Testing** | Vitest + RTL | 4x faster + React 19 ready + modern | 0KB (dev only) |

**Total Production Bundle Impact**: **~126.6KB gzipped**

- Tailwind CSS: 20KB
- Framer Motion (LazyMotion): 4.6KB
- Lottie: 82KB + animations (budgeted 100KB max per file)
- Inter font: 20KB
- SVG textures: 2-5KB
- **Total: ~128.6KB to ~130.6KB** (well within 200KB budget)

---

## Implementation Priority

### Week 1: Foundation
1. Tailwind CSS setup with Docusaurus plugin
2. shadcn/ui initialization with path aliases
3. Inter font loading via @fontsource
4. Basic grayscale theme in custom.css

### Week 2: UX Enhancement
1. Framer Motion with LazyMotion
2. Dark mode refinement and testing
3. SVG texture patterns (3 variants)
4. Component library setup (shadcn/ui components)

### Week 3: Quality Assurance
1. axe-core accessibility testing setup
2. Lighthouse CI configuration
3. Performance budgets enforcement
4. bundlesize monitoring

### Week 4: Developer Velocity
1. Vitest component testing setup
2. Test utilities and mocks
3. Coverage reporting
4. CI/CD pipeline integration

---

## Action Items

- [ ] Create plugins directory: `mkdir D:\my-web\plugins`
- [ ] Install all Week 1 dependencies
- [ ] Create Docusaurus plugins (tailwind-config.cjs, alias-plugin.cjs)
- [ ] Initialize shadcn/ui with `npx shadcn@latest init`
- [ ] Create design-system directory structure
- [ ] Set up GitHub Actions workflows (accessibility, performance, tests)
- [ ] Document component usage patterns in quickstart.md
- [ ] Create design token contracts in contracts/

---

## Risks & Mitigation

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Docusaurus + Tailwind incompatibility | Low | High | Research confirmed compatibility, plugin pattern is standard |
| Bundle size exceeds 200KB | Medium | Medium | Continuous monitoring with bundlesize + code splitting |
| React 19 library incompatibilities | Low | Medium | All recommended libraries confirmed React 19 compatible |
| Accessibility regressions | Medium | High | Automated axe-core + Lighthouse CI on every PR |
| Performance degradation | Medium | Medium | Lighthouse CI with strict budgets + animation profiling |
| Dark mode contrast failures | Medium | Medium | Automated contrast checking + manual testing |

---

## References

All research sources and documentation links are maintained in the research agent transcript (ID: a92bcf7).

---

**Next Step**: Proceed to Phase 1 (Data Model & Contracts)
