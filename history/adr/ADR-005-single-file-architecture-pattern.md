# ADR-005: Single File Architecture Pattern for Embedding Pipeline

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-11
- **Feature:** Embedding Pipeline Setup
- **Context:** Need to determine the architectural structure for the embedding pipeline that must be implemented as a single Python file due to explicit requirements. This requires balancing the constraint of single-file implementation with maintainability and modularity.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- Structure: Single file implementation (main.py) with modular function design
- Components: Distinct functions for each pipeline stage (URL crawling, text extraction, chunking, embedding, storage)
- Organization: Logical grouping of related functionality within the single file
- Dependency Management: Clear imports at the top of the file
- Execution Flow: Orchestrated through a main function

## Consequences

### Positive

- Satisfies explicit requirement for single file implementation
- Simple deployment with no complex module structure
- Easy to understand execution flow from top to bottom
- No cross-module import complexity
- Minimal file management overhead

### Negative

- Reduced maintainability as functionality grows
- Difficult to unit test individual components separately
- Potential for very long file with all functionality in one place
- Harder to navigate and understand as complexity increases
- Limited ability to reuse components in other contexts
- Potential naming conflicts as function count increases

## Alternatives Considered

Alternative A: Multi-file approach with separate modules (.py files in backend/)
- Why rejected: Explicit requirement in prompt to use only one file named main.py

Alternative B: Single file with class-based organization instead of functions
- Why rejected: Added complexity for simple pipeline, functions better match the procedural nature of the pipeline

Alternative C: Single file with submodules using local imports
- Why rejected: Still violates the single file constraint, adds unnecessary complexity

## References

- Feature Spec: specs/001-embedding-pipeline/spec.md
- Implementation Plan: specs/001-embedding-pipeline/plan.md
- Related ADRs: None
- Evaluator Evidence: specs/001-embedding-pipeline/research.md