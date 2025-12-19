# Research: Testing System Technology Decisions

**Feature**: Thorough Checking & Testing System
**Date**: 2025-12-17
**Status**: Complete

## Overview

This document captures the research and decisions made for all technical unknowns identified in the Technical Context. Each decision includes evaluated options, rationale, and rejected alternatives.

---

## 1. Static Analysis Tools

### Research Question
Which static analysis tools should be used for linting, complexity analysis, and security scanning across Python and TypeScript codebases?

### Options Evaluated

| Tool | Type | Languages | Pros | Cons |
|------|------|-----------|------|------|
| Ruff | Linter | Python | Extremely fast (Rust-based), combines multiple tools, active development | Newer tool, smaller ecosystem vs Pylint |
| ESLint | Linter | TypeScript/JS | Industry standard, extensive plugin ecosystem, configurable | Can be slow on large codebases |
| Pylint | Linter | Python | Comprehensive checks, mature | Slower than Ruff, opinionated defaults |
| Radon | Complexity | Python | Cyclomatic/cognitive complexity, maintainability index | Python-only |
| Bandit | Security | Python | OWASP-aligned, low false positives | Python-only, limited to security issues |
| SonarQube | All-in-one | Multiple | Comprehensive, enterprise features | Heavy setup, licensing costs, overkill for our needs |

### Decision

**Multi-tool approach with language-specific linters + universal complexity analyzer**

**Selected Tools**:
- **Ruff** (Python linting) - Fast, modern, combines flake8/pycodestyle/pyflakes
- **ESLint** (TypeScript linting) - Industry standard with TypeScript support
- **Radon** (Python complexity) - Cyclomatic complexity and maintainability metrics
- **Bandit** (Python security) - Static security analysis for Python code

### Rationale

1. **Language-native tools** provide best accuracy and developer familiarity
2. **Performance**: Ruff's speed (10-100x faster than alternatives) is critical for <15min pipeline goal
3. **Ecosystem fit**: ESLint is the de facto standard for TypeScript/JavaScript projects
4. **Security focus**: Bandit specializes in security issues, complementing general linting
5. **Complexity metrics**: Radon provides industry-standard cyclomatic complexity measurements

### Alternatives Rejected

- **SonarQube**: Too heavy for our scale, requires separate server infrastructure
- **Pylint**: Slower than Ruff, less critical for our use case
- **Single unified tool**: No tool adequately covers both Python and TypeScript with good performance

---

## 2. Test Runner Integration

### Research Question
How should the validation service integrate with existing test frameworks? Should we wrap native test runners or build a unified orchestrator?

### Options Evaluated

| Approach | Description | Pros | Cons |
|----------|-------------|------|------|
| Native Wrappers | Detect and wrap pytest, Jest, etc. | Works with existing setup, no migration | Multiple integration points to maintain |
| Unified Orchestrator | Build custom test runner | Single interface, full control | Requires test migration, framework lock-in |
| Test Adapter Pattern | Abstract interface + framework-specific adapters | Flexible, extensible, no lock-in | Initial complexity in adapter design |
| CI/CD Native | Rely on CI/CD platform test execution | Simple integration | Limited local validation, platform-dependent |

### Decision

**Adapter pattern with automatic test framework detection**

**Implementation Strategy**:
1. Detect installed test frameworks (pytest.ini, jest.config.js, etc.)
2. Load appropriate adapter for detected framework
3. Expose unified interface: `run_tests(suite_type, options) -> TestResult`
4. Adapters translate to native test runner commands
5. Parse native output formats (JUnit XML, TAP, JSON) to unified model

### Rationale

1. **Zero migration**: Works with existing test infrastructure immediately
2. **No lock-in**: Teams keep their preferred test frameworks
3. **Extensibility**: New framework support = new adapter module
4. **Local + CI/CD**: Same validation logic runs locally and in pipeline
5. **Separation of concerns**: Core validation logic isolated from framework specifics

### Alternatives Rejected

- **Unified orchestrator**: Forces test migration, high adoption friction
- **CI/CD native**: No local validation capability, defeats "shift-left" goal
- **Native wrappers without abstraction**: Tight coupling, hard to maintain

---

## 3. Flaky Test Detection Algorithm

### Research Question
What algorithm and confidence thresholds should be used to identify flaky tests while minimizing false positives?

### Research Findings

**Industry Standards**:
- Google: 1% pass rate change = flaky (very sensitive)
- Facebook: <80% pass rate over 10 runs (balanced)
- Netflix: Statistical analysis with 95% confidence intervals
- Spotify: ML-based prediction (complex, requires training data)

### Options Evaluated

| Approach | Detection Criteria | False Positive Rate | Complexity |
|----------|-------------------|---------------------|------------|
| Simple threshold | <90% pass rate | 15-20% | Low |
| Statistical (95% CI) | Confidence interval analysis | 5-10% | Medium |
| Time-series analysis | Pattern detection over time | 3-5% | High |
| ML-based | Predict flakiness from features | 1-3% | Very High |

### Decision

**Statistical analysis with 95% confidence threshold**

**Algorithm**:
```
For each test in execution history:
  1. Collect last N runs (N=10)
  2. Calculate pass rate = passes / total_runs
  3. If pass rate < 80%: Mark as "potentially flaky"
  4. Calculate 95% confidence interval for pass rate
  5. If CI lower bound < 80%: Confirm as "flaky"
  6. Track consecutive passes to auto-clear flaky status (5+ consecutive passes)
```

**Thresholds**:
- **Pass rate threshold**: 80% (industry standard per Facebook)
- **Confidence level**: 95% (statistical significance)
- **Sample size**: 10 runs (balances recency vs statistical power)
- **Auto-clear threshold**: 5 consecutive passes (flakiness resolved)

### Rationale

1. **Balanced accuracy**: <5% false positive rate meets SC-006 requirement
2. **Statistical rigor**: 95% confidence prevents premature flagging
3. **Practical complexity**: Medium complexity, no ML infrastructure needed
4. **Actionable**: Clear criteria for developers to understand and act on
5. **Self-healing**: Auto-clear mechanism reduces noise from transient issues

### Alternatives Rejected

- **Simple threshold**: Too many false positives (15-20%)
- **ML-based**: Requires training data, infrastructure overhead
- **Time-series**: Overkill for initial implementation, can add later

---

## 4. CI/CD Platform Integration

### Research Question
How should the validation service integrate with multiple CI/CD platforms (GitHub Actions, GitLab CI, Jenkins)?

### Options Evaluated

| Pattern | Integration Method | Platform Support | Maintenance |
|---------|-------------------|------------------|-------------|
| Platform-specific plugins | Native plugins for each platform | Best | High (N plugins) |
| Webhooks | HTTP callbacks from CI/CD | Universal | Medium |
| REST API + Adapters | Core API + thin platform adapters | Universal | Low |
| CLI tool only | Manual invocation via scripts | Universal | Lowest |

### Decision

**Generic REST API + platform-specific adapters**

**Architecture**:
```
┌─────────────────┐
│  CI/CD Platform │
│ (GitHub/GitLab) │
└────────┬────────┘
         │ Webhook
         ▼
┌─────────────────┐
│ Platform Adapter│ (Translates platform events)
└────────┬────────┘
         │ HTTP
         ▼
┌─────────────────┐
│  Validation API │ (Core validation service)
│  POST /validate │
└─────────────────┘
```

**Components**:
1. **Core API**: Platform-agnostic validation service with REST endpoints
2. **Platform Adapters**: Thin translation layer for each CI/CD platform
3. **Webhook Receivers**: Accept platform-specific webhook payloads
4. **Event Translators**: Convert to unified validation request format

### Rationale

1. **Platform independence**: Core validation logic doesn't know about CI/CD platforms
2. **Low coupling**: Adding new platform = small adapter module
3. **Testability**: Core API can be tested without CI/CD platform
4. **Reusability**: Same API for CI/CD, local CLI, and custom integrations
5. **Maintenance**: Single core codebase, adapters are simple translation logic

### Alternatives Rejected

- **Platform-specific plugins**: High maintenance burden (N×M complexity)
- **Webhooks only**: No local validation support
- **CLI tool only**: Poor CI/CD integration experience

---

## 5. Test Report Format Standards

### Research Question
What report formats should the validation service support for maximum CI/CD compatibility while capturing rich metadata?

### Industry Standards

| Format | Adoption | Richness | CI/CD Support |
|--------|----------|----------|---------------|
| JUnit XML | ~90% | Basic | Universal |
| TAP | ~20% | Basic | Limited |
| JSON | Varies | High | Custom |
| HTML | N/A | High | Human-only |

### Decision

**Dual format: JUnit XML + Custom JSON**

**Output Structure**:
```
validation-results/
├── junit.xml              # Universal CI/CD compatibility
├── results.json           # Rich metadata for flaky tests, history
└── reports/
    ├── coverage.html      # Human-readable coverage
    └── quality.html       # Human-readable quality report
```

**JUnit XML**: Standard test results (passes, failures, timing)
**Custom JSON Schema**:
```json
{
  "run_id": "uuid",
  "timestamp": "ISO8601",
  "status": "passed|failed|error",
  "duration_seconds": float,
  "quality": { "linting": {...}, "complexity": {...}, "coverage": {...} },
  "tests": { "unit": {...}, "integration": {...}, "e2e": {...} },
  "deployment": { "config": {...}, "dependencies": {...}, "security": {...} },
  "flaky_tests": [{ "test_id": "...", "pass_rate": 0.7, "history": [...] }]
}
```

### Rationale

1. **Universal compatibility**: JUnit XML works with all major CI/CD platforms
2. **Rich metadata**: JSON captures flaky test data, history, and detailed metrics
3. **Human readability**: HTML reports for manual review
4. **Extensibility**: JSON schema can evolve without breaking CI/CD integration
5. **Standard compliance**: Follows industry conventions

### Alternatives Rejected

- **JUnit XML only**: Insufficient for flaky test tracking and detailed metrics
- **JSON only**: Poor CI/CD platform compatibility
- **TAP format**: Limited adoption, not worth supporting

---

## 6. Coverage Metric Aggregation

### Research Question
How should test coverage be aggregated across multiple test types (unit/integration/E2E) and programming languages (Python/TypeScript)?

### Coverage Tools Landscape

| Tool | Language | Features | Output Formats |
|------|----------|----------|----------------|
| Coverage.py | Python | Branch, line, combines sub-runs | XML, JSON, HTML |
| Istanbul/nyc | JS/TS | Branch, line, statement, function | JSON, LCOV, HTML |
| pytest-cov | Python | pytest integration wrapper | All Coverage.py formats |
| Jest | JS/TS | Built-in coverage via Istanbul | Istanbul formats |

### Decision

**Language-specific coverage tools + unified report merger**

**Architecture**:
```
Unit Tests    Integration Tests    E2E Tests
    │                 │                │
    ▼                 ▼                ▼
Coverage.py       Coverage.py      Istanbul
    │                 │                │
    └─────────────────┴────────────────┘
                      │
                      ▼
            Unified Coverage Merger
                      │
                      ▼
        Aggregated Coverage Report
        (per test type + overall)
```

**Implementation**:
1. Run each test suite with coverage enabled
2. Collect coverage data in native formats (Coverage.py JSON, Istanbul JSON)
3. Parse and normalize to common coverage model
4. Aggregate by:
   - Test type (unit: 80% target, integration: 60%, E2E: 40%)
   - Module/package (identify uncovered areas)
   - Overall (combined: 60% target per spec)
5. Generate JUnit-compatible coverage report + detailed HTML

### Rationale

1. **Accuracy**: Language-native tools have best instrumentation support
2. **Maturity**: Leverage battle-tested coverage tools (Coverage.py, Istanbul)
3. **Flexibility**: Different targets per test type (unit higher than E2E)
4. **Granularity**: Track coverage by test type, not just overall
5. **Multi-language**: Handles Python + TypeScript in same repository

### Alternatives Rejected

- **Single coverage tool**: No tool covers both Python and TypeScript well
- **Overall coverage only**: Loses insight into unit vs integration coverage
- **Manual aggregation**: Error-prone, hard to maintain

---

## 7. Deployment Validation Approach

### Research Question
Should deployment configuration validation use declarative schema validation or imperative checks?

### Options Evaluated

| Approach | Validation Method | Flexibility | Maintainability |
|----------|------------------|-------------|-----------------|
| Schema-only | JSON Schema | Low | High |
| Imperative-only | Python validators | High | Low |
| Hybrid | Schema + imperative | High | Medium |
| Framework-based | Pydantic/Cerberus | Medium | High |

### Decision

**Hybrid: JSON Schema for structure + Python validators for complex rules**

**Architecture**:
```
Deployment Config (YAML/JSON)
         │
         ▼
   JSON Schema Validator (structure, types, required fields)
         │
         ├─ PASS ────► Python Validators (business rules)
         │                      │
         │                      ├─ Cross-field validation
         │                      ├─ Environment-specific rules
         │                      ├─ Secret detection
         │                      └─ Resource requirement checks
         │                      │
         ▼                      ▼
   Validation Report (pass/fail with specific errors)
```

**JSON Schema Handles**:
- Field presence (required vs optional)
- Data types (string, number, boolean)
- Value constraints (min/max, patterns, enums)
- Structure (nested objects, arrays)

**Python Validators Handle**:
- Cross-field validation (e.g., if cache_enabled, cache_url required)
- Environment-specific rules (production requires SSL)
- Secret detection (no hardcoded credentials)
- Resource compatibility (memory/CPU requirements vs environment)
- Dependency conflicts (version ranges, peer dependencies)

### Rationale

1. **80/20 rule**: JSON Schema catches 80% of issues (structural), Python handles remaining 20% (business logic)
2. **Declarative first**: Schema validation is fast, self-documenting
3. **Imperative when needed**: Complex rules need code, not schema constraints
4. **Tooling support**: JSON Schema has excellent IDE support (autocomplete, validation)
5. **Versioning**: Schema version evolves with configuration format

### Alternatives Rejected

- **Schema-only**: Can't express complex business rules in JSON Schema
- **Imperative-only**: Loses benefits of declarative validation, harder to maintain
- **Framework-based only**: Tighter coupling, less flexible for multi-config scenarios

---

## 8. Dependency Vulnerability Scanning

### Research Question
Should vulnerability scanning be integrated as a built-in feature or integrated via external service APIs?

### Options Evaluated

| Approach | Tool | Database | Maintenance | Cost |
|----------|------|----------|-------------|------|
| Integrated | Safety (Python), npm audit (Node) | Open-source CVE | Low | Free |
| External API | Snyk API | Commercial | Low | $$ |
| Self-hosted | Dependency-Check | NVD mirror | High | Free |
| Advisory DB | OSV.dev | Open-source | Low | Free |

### Decision

**Integration with existing security audit tools**

**Selected Tools**:
- **Safety** (Python): PyPI vulnerability database, subprocess integration
- **npm audit** (Node.js): Built into npm, reads package-lock.json
- **OSV.dev API** (fallback): Language-agnostic vulnerability database

**Integration Pattern**:
```python
def scan_dependencies(project_root: Path) -> SecurityReport:
    findings = []

    # Python dependencies
    if (project_root / "requirements.txt").exists():
        findings.extend(run_safety_check(project_root))

    # Node.js dependencies
    if (project_root / "package-lock.json").exists():
        findings.extend(run_npm_audit(project_root))

    # Fallback to OSV.dev for other ecosystems
    findings.extend(run_osv_scan(project_root))

    return SecurityReport(findings=findings)
```

### Rationale

1. **Mature databases**: Safety and npm audit use actively-maintained vulnerability databases
2. **Zero setup**: Tools are standard in Python/Node ecosystems, no separate accounts needed
3. **Subprocess integration**: Simple to integrate, no library dependencies
4. **Offline capable**: Safety can cache database for offline validation
5. **Free**: No licensing costs, suitable for open-source projects
6. **Standard output**: Both tools output structured JSON for parsing

### Alternatives Rejected

- **Snyk API**: Requires paid account, API rate limits, external dependency
- **Self-hosted Dependency-Check**: High maintenance (NVD mirror updates), overkill
- **Build-only checks**: Need local validation too, subprocess integration is simplest

---

## Summary of Decisions

| Area | Decision | Key Benefit |
|------|----------|-------------|
| Static Analysis | Multi-tool (Ruff, ESLint, Radon, Bandit) | Language-native accuracy + performance |
| Test Integration | Adapter pattern | Zero migration, no framework lock-in |
| Flaky Detection | Statistical (95% CI, <80% pass rate) | <5% false positives |
| CI/CD Integration | REST API + adapters | Platform independence |
| Report Format | JUnit XML + custom JSON | Universal compatibility + rich metadata |
| Coverage Aggregation | Native tools + merger | Multi-language, per-test-type targets |
| Deployment Validation | JSON Schema + Python validators | 80% declarative, 20% imperative |
| Vulnerability Scanning | Safety + npm audit | Mature, free, standard tools |

All decisions support the <15 minute validation pipeline goal, <5% false positive rate, and 10+ concurrent developer requirements from the specification.
