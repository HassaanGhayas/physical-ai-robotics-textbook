# Implementation Plan: UI/UX Design System Enhancement

**Branch**: `001-ui-ux-design` | **Date**: 2025-12-18 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ui-ux-design/spec.md`

## Summary

Implement a modern, accessible design system for the Physical AI & Humanoid Robotics documentation site with a clean black-and-white grayscale theme, Inter typography, bento grid layouts, smooth animations, and shadcn/ui component library integration. The system must achieve WCAG AA accessibility compliance (Lighthouse 90+), maintain high performance (FCP <1.5s, LCP <2.5s, 60fps animations), and support responsive layouts across all device sizes. This enhancement will transform the existing Docusaurus site from default styling to a professional, spacious design with textured backgrounds, fade-in animations, Lottie micro-interactions, and comprehensive keyboard navigation support.

## Technical Context

**Language/Version**: TypeScript 5.6.2 / React 19.0 / Node.js 20+
**Primary Dependencies**: Docusaurus 3.9.2, Tailwind CSS 4.x (to be added), shadcn/ui, Lottie-react, Framer Motion, clsx
**Storage**: N/A (static site generator)
**Testing**: Jest + React Testing Library (to be added), Playwright for E2E, Lighthouse CI, axe-core for accessibility
**Target Platform**: Web browsers (Chrome 90+, Firefox 88+, Safari 14+, Edge 90+), responsive design (mobile-first)
**Project Type**: Web application (Docusaurus static site)
**Performance Goals**:
- First Contentful Paint (FCP) < 1.5 seconds
- Largest Contentful Paint (LCP) < 2.5 seconds
- Cumulative Layout Shift (CLS) < 0.1
- All animations at 60fps
- Lighthouse Performance score 90+
- Lighthouse Accessibility score 90+

**Constraints**:
- Must maintain Docusaurus compatibility (no breaking changes to content authoring)
- Maximum Lottie animation file size: 100KB per file
- Inter font load time < 100ms (font-display: swap)
- All animations must respect `prefers-reduced-motion`
- WCAG AA compliance mandatory (4.5:1 contrast for body text, 3:1 for large text)
- Minimal bundle size increase (<200KB gzipped for design system)

**Scale/Scope**:
- ~9 main documentation modules (Hardware Requirements, Introduction, ROS 2, Control Systems, AI Integration, Digital Twin, AI-Robot Brain, VLA, Assessments)
- ~30+ individual documentation pages
- Homepage with bento grid layout
- Chatbot component integration
- Navigation, sidebar, and footer components

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Compliance with Constitution

**Spec-Driven Development**: ✅ PASS
- Following Spec-Kit Plus methodology with complete specification in `specs/001-ui-ux-design/spec.md`
- All requirements documented and validated before implementation

**Modular Architecture**: ✅ PASS
- Design system implemented as separate concerns: theme configuration, component library, animation system, accessibility layer
- Docusaurus plugin architecture preserved
- No coupling between design system and content

**AI Integration First**: ✅ PASS (No Impact)
- Design system enhancement does not affect existing RAG chatbot functionality
- Chatbot component will be restyled to match new design system
- No changes to Cohere Models, OpenAI Agents SDK, or RAG backend

**Authentication & Personalization**: ✅ PASS (No Impact)
- Design system neutral to Better Auth implementation
- Authentication UI components will benefit from shadcn/ui styling
- Urdu translation button will be restyled with new design tokens

**Performance & Scalability**: ✅ PASS with Vigilance Required
- Design system adds performance budget: <200KB gzipped
- Lottie animations capped at 100KB each, with fallbacks
- Font loading optimized with `font-display: swap`
- Animation performance monitored (60fps requirement)
- **VIGILANCE**: Must validate bundle size and performance metrics continuously

**Multi-language Support**: ✅ PASS (No Impact)
- Design system language-agnostic
- Inter font supports Latin and extended character sets
- Urdu translation feature unaffected

**Technology Stack Requirements**: ✅ PASS with Additions
- Preserves: Docusaurus 3.9.2, React 19.0, GitHub Pages deployment
- Adds: Tailwind CSS, shadcn/ui, Lottie-react, Framer Motion
- All additions are frontend-only, no backend changes
- **JUSTIFICATION**: New dependencies required for design system requirements (spec FR-010, FR-006, FR-004)

**Development Workflow**: ✅ PASS
- Follows sequence: UI Customization (current phase) → UX Customization → GitHub Push → GitHub Pages Deployment → Testing
- Fits after "Writing Book content" phase (already complete)
- Precedes RAG Chatbot Development enhancement

### Gate Evaluation: **PASS**

All constitution principles satisfied. Performance vigilance required during implementation (continuous monitoring of bundle size, animation performance, and Lighthouse scores).

## Project Structure

### Documentation (this feature)

```text
specs/001-ui-ux-design/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (technology decisions)
├── data-model.md        # Phase 1 output (design tokens, theme config)
├── quickstart.md        # Phase 1 output (developer guide)
├── contracts/           # Phase 1 output (component APIs, theme schema)
│   ├── theme-schema.json
│   ├── design-tokens.json
│   └── component-api.ts
├── checklists/
│   └── requirements.md  # Specification quality validation
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Docusaurus web application structure (existing + additions)
src/
├── components/          # Existing React components
│   ├── ChatBot/        # Existing chatbot (to be restyled)
│   ├── HomepageFeatures/
│   └── [new design system components to be added]
├── css/                 # Existing custom CSS
│   └── custom.css      # Will be migrated to Tailwind
├── lib/                 # Existing utilities
│   └── api/            # API client for chatbot
├── pages/              # Existing pages
│   └── index.tsx       # Homepage (will add bento grid)
├── theme/              # Docusaurus theme overrides (existing)
│   └── [to be expanded with design system]
└── types/              # TypeScript types

# New additions for design system:
src/
├── design-system/      # NEW: Design system core
│   ├── tokens/         # Design tokens (colors, spacing, typography)
│   ├── animations/     # Animation utilities and configs
│   ├── components/     # shadcn/ui components
│   └── hooks/          # Custom React hooks (useReducedMotion, etc.)
├── assets/             # NEW: Static assets
│   ├── fonts/          # Inter font files (woff2)
│   ├── animations/     # Lottie JSON files
│   └── textures/       # Background texture patterns (SVG/PNG)

# Configuration files (root):
tailwind.config.js      # NEW: Tailwind configuration with grayscale theme
postcss.config.js       # NEW: PostCSS for Tailwind
components.json         # NEW: shadcn/ui configuration

# Testing structure:
tests/                  # NEW: Test directory
├── accessibility/      # axe-core accessibility tests
├── performance/        # Lighthouse CI tests
├── visual-regression/  # Visual regression tests (optional)
└── e2e/               # Playwright end-to-end tests

# Backend (existing, no changes):
backend/
├── api.py             # RAG backend (existing, no changes)
└── [other backend files]

docs/                   # Existing documentation content (no structure changes)
├── book/              # Book content modules
static/                # Existing static assets
```

**Structure Decision**: Docusaurus web application structure with new `src/design-system/` directory for design system core. This maintains clear separation between design system implementation and Docusaurus framework code while allowing easy import of design tokens and components throughout the application. The design system will be implemented as a self-contained module that can be imported by any component.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations. All gates passed.

## Phase 0: Research & Technology Decisions

### Research Tasks

The following technology decisions must be researched and documented in `research.md`:

1. **Tailwind CSS Version Selection**
   - **Question**: Tailwind CSS v3 (stable) vs v4 (alpha) - which version to use?
   - **Research needed**: Docusaurus compatibility, feature parity, stability concerns
   - **Decision criteria**: Docusaurus 3.9.2 compatibility, grayscale theme support, PostCSS integration

2. **shadcn/ui Integration Strategy**
   - **Question**: How to integrate shadcn/ui with Docusaurus + React 19?
   - **Research needed**: Installation method (CLI vs manual), Docusaurus compatibility, React 19 compatibility
   - **Decision criteria**: Works with Docusaurus build system, supports SSR/SSG, minimal configuration

3. **Animation Library Selection**
   - **Question**: Framer Motion vs React Spring vs pure CSS animations?
   - **Research needed**: Performance comparison, bundle size, SSR compatibility, `prefers-reduced-motion` support
   - **Decision criteria**: Best performance (60fps), small bundle size, easy `prefers-reduced-motion` integration

4. **Lottie-react vs lottie-web**
   - **Question**: Which Lottie library for React integration?
   - **Research needed**: Bundle size comparison, React 19 compatibility, performance differences
   - **Decision criteria**: Smallest bundle size while maintaining functionality

5. **Font Loading Strategy**
   - **Question**: Google Fonts CDN vs self-hosted Inter font?
   - **Research needed**: Performance impact, GDPR compliance, Docusaurus static build compatibility
   - **Decision criteria**: Fastest load time (<100ms), works with static site generation, GDPR compliant

6. **Texture Pattern Implementation**
   - **Question**: SVG patterns vs PNG images vs CSS-only patterns?
   - **Research needed**: Performance on low-end devices, scalability, file size
   - **Decision criteria**: Best performance, smallest file size, works with SSG

7. **Accessibility Testing Strategy**
   - **Question**: Which automated accessibility testing tools to integrate?
   - **Research needed**: axe-core vs Pa11y vs Lighthouse CI
   - **Decision criteria**: Most comprehensive coverage, CI/CD integration, catches WCAG AA violations

8. **Dark Mode Implementation**
   - **Question**: How to implement dark mode with grayscale theme in Docusaurus?
   - **Research needed**: Docusaurus dark mode API, CSS custom properties approach, shadcn/ui dark mode
   - **Decision criteria**: Works with Docusaurus theme switcher, maintains contrast ratios, easy to toggle

9. **Performance Monitoring**
   - **Question**: How to enforce performance budgets in CI/CD?
   - **Research needed**: Lighthouse CI integration, bundle size monitoring tools
   - **Decision criteria**: Prevents regressions, fails build on violations, easy to set up

10. **Component Testing Strategy**
    - **Question**: Jest + React Testing Library or Vitest?
    - **Research needed**: Docusaurus compatibility, React 19 support, performance
    - **Decision criteria**: Works with Docusaurus build, fastest test runs, best React 19 support

### Research Output

Each research task will produce a decision entry in `research.md` with:
- **Decision**: What was chosen
- **Rationale**: Why chosen (performance, compatibility, bundle size)
- **Alternatives Considered**: What else was evaluated
- **Tradeoffs**: What we're giving up
- **Action Items**: Installation steps, configuration needed

## Phase 1: Design & Contracts

### Data Model: Design Tokens & Theme Configuration

`data-model.md` will document:

1. **Design Token Schema**
   - Color tokens: grayscale-0 through grayscale-1000, semantic tokens (text-primary, bg-primary)
   - Spacing tokens: 4px scale (4, 8, 12, 16, 24, 32, 48, 64, 96)
   - Typography tokens: font-family, font-sizes (14-64px), font-weights, line-heights
   - Border radius tokens: sm (8px), md (12px), lg (16px)
   - Shadow tokens: elevation-1 through elevation-5
   - Animation tokens: duration-fast (100ms), duration-normal (200ms), duration-slow (300-500ms), easing functions

2. **Theme Configuration Structure**
   ```typescript
   interface ThemeConfig {
     colors: GrayscaleColorPalette;
     spacing: SpacingScale;
     typography: TypographyScale;
     borderRadius: BorderRadiusScale;
     shadows: ShadowScale;
     animations: AnimationConfig;
   }
   ```

3. **Component State Mappings**
   - Default state styling
   - Hover state styling
   - Focus state styling (WCAG 3:1 contrast requirement)
   - Disabled state styling
   - Active state styling

4. **Responsive Breakpoints**
   - Mobile: 0-767px (single column)
   - Tablet: 768-1199px (2 columns for bento grid)
   - Desktop: 1200px+ (3 columns for bento grid)

### API Contracts

`contracts/` directory will contain:

1. **`theme-schema.json`**: JSON schema for theme configuration validation
   - Ensures all required tokens are defined
   - Validates color contrast ratios
   - Validates spacing scale consistency

2. **`design-tokens.json`**: Complete design token export
   - All grayscale color values with hex codes
   - All spacing values in px and rem
   - All typography definitions
   - Can be imported by any tool (Figma, Storybook, etc.)

3. **`component-api.ts`**: TypeScript interfaces for all components
   ```typescript
   // Example: BentoGrid component API
   interface BentoGridProps {
     items: BentoGridItem[];
     columns?: { mobile: number; tablet: number; desktop: number };
     gap?: SpacingToken;
     className?: string;
   }

   interface BentoGridItem {
     id: string;
     content: React.ReactNode;
     span?: { mobile: number; tablet: number; desktop: number };
     aspectRatio?: string;
   }
   ```

4. **Animation Contracts**
   ```typescript
   interface FadeInConfig {
     duration: number; // 300-500ms
     delay?: number;
     translateY?: number; // 8-16px
     easing: 'ease-out';
     respectReducedMotion: true;
   }

   interface LottieAnimationProps {
     animationData: object;
     loop?: boolean;
     autoplay?: boolean;
     maxSize?: number; // 100KB limit
     fallbackImage?: string;
   }
   ```

### Quickstart Guide

`quickstart.md` will provide:

1. **Installation Steps**
   ```bash
   # Install dependencies
   npm install tailwindcss postcss autoprefixer
   npx tailwindcss init -p
   npm install @shadcn/ui
   npm install lottie-react framer-motion
   npm install @fontsource/inter
   ```

2. **Configuration Steps**
   - Tailwind CSS setup with Docusaurus
   - shadcn/ui initialization
   - Theme token import
   - Font loading configuration

3. **Usage Examples**
   ```tsx
   // Using design tokens
   import { tokens } from '@/design-system/tokens';

   // Using shadcn/ui components
   import { Button } from '@/design-system/components/ui/button';

   // Using animations
   import { FadeIn } from '@/design-system/animations';

   // Using accessibility hooks
   import { useReducedMotion } from '@/design-system/hooks';
   ```

4. **Common Patterns**
   - Creating a new component with grayscale theme
   - Adding fade-in animation to a section
   - Implementing keyboard navigation
   - Adding Lottie animation with fallback

5. **Testing Guidelines**
   - Running accessibility tests
   - Checking performance metrics
   - Validating contrast ratios
   - Testing keyboard navigation

### Agent Context Update

After completing Phase 1, run:
```bash
powershell -ExecutionPolicy Bypass -File ".specify/scripts/powershell/update-agent-context.ps1" -AgentType claude
```

This will update `.claude/CLAUDE.md` with:
- New dependencies: Tailwind CSS, shadcn/ui, Lottie-react, Framer Motion
- Design system architecture overview
- Design token usage patterns
- Component testing requirements
- Performance budgets and monitoring

## Post-Phase 1 Constitution Re-check

After completing Phase 1 design work, re-evaluate constitution compliance:

**Performance & Scalability**: ⚠️ Requires Validation
- Design system bundle size must be measured
- Lighthouse scores must be validated
- Animation performance must be profiled
- Font loading performance must be tested

**Action Required**: Add performance validation tests to tasks.md
- Bundle size analysis
- Lighthouse CI integration
- Animation FPS profiling
- Font loading measurement

All other constitution principles remain compliant.

## Next Steps

After completing `/sp.plan`:

1. **Review Generated Artifacts**:
   - `research.md` - Technology decisions and rationale
   - `data-model.md` - Design tokens and theme configuration
   - `contracts/` - Component APIs and schemas
   - `quickstart.md` - Developer onboarding guide

2. **Run `/sp.tasks`** to generate implementation task breakdown:
   - Task 1: Install dependencies and configure build system
   - Task 2: Implement design token system
   - Task 3: Set up shadcn/ui components
   - Task 4: Implement Inter font loading
   - Task 5: Create textured background system
   - Task 6: Build bento grid layout components
   - Task 7: Implement fade-in animations
   - Task 8: Integrate Lottie animations with fallbacks
   - Task 9: Implement accessibility features (skip links, focus management)
   - Task 10: Add keyboard navigation enhancements
   - Task 11: Create dark mode toggle
   - Task 12: Set up testing infrastructure (accessibility, performance)
   - Task 13: Migrate existing components to new design system
   - Task 14: Performance optimization and validation

3. **Begin Implementation** following Red-Green-Refactor TDD cycle:
   - Red: Write failing tests for each requirement
   - Green: Implement minimum code to pass tests
   - Refactor: Clean up and optimize

## Architectural Decisions Requiring ADRs

The following significant architectural decisions should be documented as ADRs after planning:

1. **Design System Architecture**: Separate `src/design-system/` module vs inline with components
   - **Impact**: Affects maintainability, reusability, testing strategy
   - **Alternatives**: Inline styling, CSS modules, styled-components
   - **Rationale**: Centralized design system enables consistency and easy updates

2. **Animation Library Choice**: Framer Motion vs React Spring vs CSS
   - **Impact**: Bundle size, performance, developer experience
   - **Alternatives**: Different animation libraries or pure CSS
   - **Rationale**: Based on research phase findings

3. **Font Loading Strategy**: Self-hosted vs CDN
   - **Impact**: Performance, GDPR compliance, hosting costs
   - **Alternatives**: Google Fonts CDN, Adobe Fonts, system fonts
   - **Rationale**: Based on research phase findings

Run `/sp.adr` after research phase to create these ADRs.

## Success Metrics Tracking

During implementation, track these metrics to validate against success criteria:

| Success Criterion | Target | Measurement Method | Tracking Frequency |
|-------------------|--------|-------------------|-------------------|
| SC-001: Lighthouse Accessibility | 90+ | Lighthouse CI | Every commit |
| SC-002: WCAG AA Contrast | 4.5:1 / 3:1 | axe-core | Every commit |
| SC-003: FCP / LCP | <1.5s / <2.5s | Lighthouse CI | Every commit |
| SC-004: Keyboard Navigation | 100% | Manual + Playwright | Every PR |
| SC-005: Animation Performance | 60fps, CLS <0.1 | Chrome DevTools | Every animation |
| SC-006: Font Load Time | <100ms | Network panel | Every font change |
| SC-007: Alt Text Coverage | 100% | axe-core | Every image addition |
| SC-008: Reduced Motion | <100ms or disabled | Manual testing | Every animation |
| SC-009: Mobile Usability | 44px tap targets | Chrome DevTools | Every component |
| SC-010: Responsive Breakpoints | 1/2/3 columns | Visual testing | Every layout change |
| SC-011: Design System Docs | Exists | Manual review | Before launch |
| SC-012: Dev Velocity | 50% faster | Time tracking | Post-launch |

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation Strategy |
|------|-----------|--------|-------------------|
| Docusaurus + Tailwind CSS compatibility issues | Medium | High | Research phase validates compatibility, have fallback to CSS modules |
| Bundle size exceeds budget (>200KB) | Medium | Medium | Code splitting, lazy loading, tree shaking, continuous monitoring |
| Accessibility regressions | Low | High | Automated testing (axe-core), manual keyboard testing every PR |
| Performance degradation (animations) | Medium | Medium | Animation performance profiling, `prefers-reduced-motion` fallbacks |
| React 19 compatibility with libraries | Low | Medium | Research phase validates all library versions |
| Dark mode contrast ratio failures | Medium | Medium | Automated contrast checking, invert grayscale with validation |
| Lottie animation file sizes exceed limit | Low | Low | Optimize animations, set up file size checks in CI |
| Inter font loading causes FOUT | Low | Low | `font-display: swap`, preload critical fonts |

## Appendix: Implementation Priorities

Based on spec user story priorities:

### Priority 1 (P1) - Foundation [Week 1-2]
- Visual Identity & Typography
- Inter font loading
- Grayscale color system
- Textured background patterns
- Basic theme configuration

### Priority 2 (P2) - Layout & Accessibility [Week 2-3]
- Spacious layout system
- Bento grid components
- Enhanced accessibility features
- Keyboard navigation
- Screen reader support
- ARIA implementation

### Priority 3 (P3) - Polish & Components [Week 3-4]
- Smooth animations & micro-interactions
- Lottie animation integration
- Component library (shadcn/ui) setup
- Dark mode implementation
- Performance optimization

This phased approach allows for:
- Early validation of design foundation
- Accessibility built into layout from start
- Polish added without blocking core functionality
- Each phase independently testable and deployable
