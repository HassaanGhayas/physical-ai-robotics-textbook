# ADR-003: Deployment Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** Book Creation System
- **Context:** Need to determine how the book creation system will be deployed and hosted to ensure availability, performance, and cost-effectiveness for readers.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Hosting: GitHub Pages (free, reliable hosting)
- Deployment: Static site generation with automated GitHub Actions
- Domain: Support for custom domains
- Build Process: npm run deploy script integrated with Git workflow
- CDN: GitHub's global CDN for content delivery

## Consequences

### Positive

- Cost-effective (free hosting)
- Integrated with Git workflow
- Reliable and scalable infrastructure
- Custom domain support
- Automatic SSL certificates
- Global CDN for fast content delivery
- Simple deployment process

### Negative

- Limited server-side functionality
- Deployment constraints (no server-side rendering after build)
- Limited customization of server configuration
- Potential limitations with very high traffic
- Dependency on GitHub's platform availability

## Alternatives Considered

Alternative A: Vercel hosting
- Why rejected: Additional complexity for static site, cost considerations

Alternative B: AWS S3 + CloudFront
- Why rejected: More complex setup, ongoing costs, unnecessary complexity for this use case

Alternative C: Netlify hosting
- Why rejected: Additional external dependency, GitHub Pages sufficient for requirements

## References

- Feature Spec: specs/001-book-creation/spec.md
- Implementation Plan: specs/001-book-creation/plan.md
- Related ADRs: ADR-001-technology-stack-and-architecture.md
- Evaluator Evidence: specs/001-book-creation/research.md