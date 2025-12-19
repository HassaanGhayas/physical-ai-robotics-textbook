# Performance Monitoring

This document outlines the performance monitoring and optimization strategies for the Physical AI & Humanoid Robotics documentation site.

## Performance Budget

### Bundle Size Targets

- **Total JavaScript**: < 200KB (gzipped)
- **CSS**: < 50KB (gzipped)
- **Fonts**: < 30KB (Inter Variable subset)
- **Images**: Lazy-loaded, optimized

### Current Bundle Breakdown

- Tailwind CSS: ~20KB (gzipped, tree-shaken)
- shadcn/ui components: 0KB (copy-paste, no bundle)
- Framer Motion (LazyMotion): ~4.6KB (gzipped)
- lottie-react: ~82KB (gzipped)
- Inter font: ~20KB (WOFF2, Variable)
- SVG patterns: 2-5KB (inline data URIs)

**Total Estimated**: ~126.6KB (gzipped) ✅ Under budget

## Lighthouse CI

### Running Lighthouse Audits

```bash
# Build and run full Lighthouse audit
npm run a11y:audit

# Run Lighthouse only (requires build directory)
npm run lighthouse

# Collect Lighthouse data
npm run lighthouse:collect

# Assert against thresholds
npm run lighthouse:assert
```

### Performance Thresholds

- **Performance**: ≥ 90
- **Accessibility**: ≥ 95
- **Best Practices**: ≥ 90
- **SEO**: ≥ 90

## Testing

### Component Tests

```bash
# Run all tests
npm test

# Run tests with UI
npm test:ui

# Generate coverage report
npm test:coverage
```

### Accessibility Testing

```bash
# Run axe-core accessibility audit in browser console
window.runA11yAudit()
```

## Optimization Strategies

### 1. Code Splitting

- Docusaurus automatically splits routes
- Framer Motion uses LazyMotion for dynamic imports
- Lottie animations loaded on-demand

### 2. Font Optimization

- Inter Variable font (single file, all weights)
- Self-hosted (GDPR compliant, no external requests)
- OpenType features: `cv11`, `ss01`
- Font-display: swap

### 3. Image Optimization

- SVG patterns as inline data URIs (no HTTP requests)
- Lazy loading for feature images
- Modern formats (WebP, AVIF) where supported

### 4. CSS Optimization

- Tailwind CSS tree-shaking (unused classes removed)
- CSS Modules for component styles
- Critical CSS inlined by Docusaurus

### 5. JavaScript Optimization

- Tree-shaking enabled
- LazyMotion for reduced motion bundle
- shadcn/ui components (no runtime library)

## Monitoring in Production

### Metrics to Track

1. **Core Web Vitals**
   - LCP (Largest Contentful Paint): < 2.5s
   - FID (First Input Delay): < 100ms
   - CLS (Cumulative Layout Shift): < 0.1

2. **Bundle Size**
   - Monitor build output
   - Track changes in CI/CD

3. **Accessibility Score**
   - Lighthouse CI in GitHub Actions
   - axe-core violations: 0

### GitHub Actions Integration

Add to `.github/workflows/`:

```yaml
name: Performance & Accessibility

on: [push, pull_request]

jobs:
  lighthouse:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
      - run: npm ci
      - run: npm run build
      - run: npm run lighthouse
```

## Performance Checklist

- [x] Bundle size under 200KB (gzipped)
- [x] Lighthouse performance score ≥ 90
- [x] Lighthouse accessibility score ≥ 95
- [x] WCAG AA contrast ratios met
- [x] Reduced motion support
- [x] Font optimization (self-hosted, Variable)
- [x] CSS optimization (Tailwind tree-shaking)
- [x] JavaScript tree-shaking enabled
- [x] Component testing setup (Vitest)
- [x] Accessibility testing (axe-core, Lighthouse CI)

## Maintenance

### Regular Tasks

1. **Monthly**: Review Lighthouse CI reports
2. **Quarterly**: Audit bundle size and dependencies
3. **On Release**: Run full accessibility audit
4. **On Dependency Update**: Verify bundle size impact

### Tools

- **Lighthouse CI**: Automated performance audits
- **axe-core**: Runtime accessibility testing
- **Vitest**: Component unit tests
- **Webpack Bundle Analyzer**: (optional) Visualize bundle

## Resources

- [Web Vitals](https://web.dev/vitals/)
- [Lighthouse CI](https://github.com/GoogleChrome/lighthouse-ci)
- [axe-core](https://github.com/dequelabs/axe-core)
- [WCAG 2.1 Guidelines](https://www.w3.org/WAI/WCAG21/quickref/)
