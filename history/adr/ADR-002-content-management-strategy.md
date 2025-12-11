# ADR-002: Content Management Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** Book Creation System
- **Context:** Need to determine how book content will be structured, stored, and managed to support hierarchical organization (books, chapters, sections) with version control and easy editing.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Storage: File-based (Markdown content stored in repository)
- Structure: Hierarchical organization with docs/book/chapter/section pattern
- Format: Markdown for content with frontmatter for metadata
- Navigation: Sidebar configuration for book structure
- Content Types: Support for text, code blocks, images, equations, and other media

## Consequences

### Positive

- Simple, version-controllable content management
- Clear hierarchical organization matching book structure
- Easy editing with standard text editors
- Git-based version control and collaboration
- Good SEO with static content
- Easy backup and migration

### Negative

- Potential performance issues with very large files
- Manual organization required for complex book structures
- Less sophisticated content management than database solutions
- Potential merge conflicts with concurrent edits
- Limited content relationships compared to database solutions

## Alternatives Considered

Alternative A: Database-backed content management (PostgreSQL + CMS)
- Why rejected: More complex architecture, additional infrastructure, unnecessary for static book content

Alternative B: Headless CMS (Contentful, Strapi, etc.)
- Why rejected: Additional external dependency, subscription costs, more complex deployment

Alternative C: Wiki-style content management system
- Why rejected: Less structured, doesn't align with book format requirements

## References

- Feature Spec: specs/001-book-creation/spec.md
- Implementation Plan: specs/001-book-creation/plan.md
- Related ADRs: ADR-001-technology-stack-and-architecture.md
- Evaluator Evidence: specs/001-book-creation/research.md