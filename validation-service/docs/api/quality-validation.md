# Quality Validation API Documentation

This document describes the API endpoints for code quality validation (User Story 1).

## Overview

The quality validation API provides endpoints for triggering code quality checks and retrieving results. Quality validation includes:
- **Linting**: Style violations detected using Ruff
- **Complexity**: Cyclomatic complexity analysis using Radon
- **Coverage**: Test coverage measurement using coverage.py
- **Security**: Vulnerability scanning using Bandit

## Base URL

- **Local Development**: `http://localhost:8080/api/v1`
- **Production**: `https://validation.example.com/api/v1`

---

## Endpoints

### POST /validate/quality

Triggers code quality validation for a project.

#### Request

```json
{
  "repository": "git@github.com:user/repo.git",
  "branch": "main",
  "commit_sha": "a1b2c3d4e5f6",
  "trigger_source": "manual",
  "user": "developer@example.com"
}
```

#### Request Fields

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `repository` | string | Yes | Repository path or URL |
| `branch` | string | Yes | Git branch name |
| `commit_sha` | string | Yes | Git commit hash |
| `trigger_source` | enum | No | One of: `manual`, `pre_commit`, `ci_cd`, `scheduled`. Default: `manual` |
| `user` | string | No | User triggering validation |

#### Response (202 Accepted)

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "pending",
  "message": "Quality validation accepted and queued",
  "estimated_duration_seconds": 120.0
}
```

#### Error Responses

| Status | Description |
|--------|-------------|
| 400 | Invalid request parameters |
| 503 | Service unavailable (too many concurrent validations) |

---

### GET /results/{run_id}

Retrieves complete validation results for a specific run.

#### Path Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `run_id` | UUID | Validation run identifier |

#### Response (200 OK)

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "timestamp": "2025-12-17T10:30:00Z",
  "trigger_source": "manual",
  "status": "passed",
  "started_at": "2025-12-17T10:30:00Z",
  "completed_at": "2025-12-17T10:32:00Z",
  "duration_seconds": 120.5,
  "repository": "git@github.com:user/repo.git",
  "branch": "main",
  "commit_sha": "a1b2c3d4e5f6",
  "quality_report": {
    "report_id": "660e8400-e29b-41d4-a716-446655440001",
    "run_id": "550e8400-e29b-41d4-a716-446655440000",
    "timestamp": "2025-12-17T10:32:00Z",
    "status": "passed",
    "linting": {
      "violations": [],
      "total_violations": 0,
      "error_count": 0,
      "warning_count": 0
    },
    "complexity": {
      "average_complexity": 5.2,
      "max_complexity": 12,
      "high_complexity_functions": []
    },
    "coverage": {
      "line_coverage_percent": 85.5,
      "branch_coverage_percent": 75.0,
      "uncovered_lines": 150,
      "by_test_type": {
        "unit": 70.0,
        "integration": 12.5,
        "e2e": 3.0
      }
    },
    "security": {
      "vulnerabilities": [],
      "critical_count": 0,
      "high_count": 0,
      "medium_count": 0,
      "low_count": 0
    },
    "summary": {
      "maintainability_index": 78.5,
      "quality_gate_passed": true
    }
  }
}
```

#### Error Responses

| Status | Description |
|--------|-------------|
| 404 | Run not found |

---

### GET /results/{run_id}/status

Lightweight endpoint to check validation progress.

#### Response (200 OK)

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "running",
  "progress_percent": 45.0,
  "current_phase": "quality"
}
```

---

## Data Structures

### Violation

Represents a code style violation detected by linting.

```json
{
  "file": "src/main.py",
  "line": 10,
  "column": 5,
  "rule": "E501",
  "severity": "error",
  "message": "Line too long (120 > 88 characters)"
}
```

| Field | Type | Description |
|-------|------|-------------|
| `file` | string | File path relative to repository root |
| `line` | integer | Line number (1-indexed) |
| `column` | integer | Column number (optional) |
| `rule` | string | Linting rule identifier (e.g., E501, F401) |
| `severity` | enum | One of: `critical`, `high`, `medium`, `low`, `info`, `error`, `warning` |
| `message` | string | Human-readable description |

### Vulnerability

Represents a security vulnerability in dependencies.

```json
{
  "package": "requests",
  "version": "2.25.0",
  "severity": "high",
  "cve_id": "CVE-2021-12345",
  "description": "Security issue in requests library",
  "fixed_version": "2.26.0"
}
```

### CodeQualityReport

Complete quality validation results.

| Field | Type | Description |
|-------|------|-------------|
| `report_id` | UUID | Unique report identifier |
| `run_id` | UUID | Parent validation run |
| `timestamp` | datetime | When analysis completed |
| `status` | enum | `passed`, `failed`, or `warning` |
| `linting` | LintingResults | Style violation results |
| `complexity` | ComplexityResults | Code complexity metrics |
| `coverage` | CoverageResults | Test coverage metrics |
| `security` | SecurityResults | Security scan results |
| `summary` | QualitySummary | Aggregated quality metrics |

---

## Status Values

### ValidationStatus

| Value | Description |
|-------|-------------|
| `pending` | Validation queued, not started |
| `running` | Validation in progress |
| `passed` | All checks passed |
| `failed` | One or more checks failed |
| `error` | Validation encountered an error |
| `timeout` | Validation exceeded time limit |

### CheckStatus

| Value | Description |
|-------|-------------|
| `passed` | Check completed successfully |
| `failed` | Check found issues |
| `warning` | Check found non-blocking issues |

### Severity

| Value | Description |
|-------|-------------|
| `critical` | Must be fixed before deployment |
| `high` | Should be fixed soon |
| `medium` | Recommended to fix |
| `low` | Minor issue |
| `info` | Informational only |
| `error` | Code error (linting) |
| `warning` | Code warning (linting) |

---

## Examples

### Running Quality Validation

```bash
# Trigger validation
curl -X POST http://localhost:8080/api/v1/validate/quality \
  -H "Content-Type: application/json" \
  -d '{
    "repository": "/path/to/project",
    "branch": "feature-branch",
    "commit_sha": "abc123"
  }'

# Check status
curl http://localhost:8080/api/v1/results/550e8400-e29b-41d4-a716-446655440000/status

# Get full results
curl http://localhost:8080/api/v1/results/550e8400-e29b-41d4-a716-446655440000
```

### Using Python Client

```python
from src.quality.validator import QualityValidator
from pathlib import Path
from uuid import uuid4

validator = QualityValidator()
run_id = uuid4()

report = validator.validate(
    project_path=Path("/path/to/project"),
    run_id=run_id,
)

if report.status == CheckStatus.PASSED:
    print("Quality validation passed!")
else:
    print(f"Issues found: {report.linting.total_violations} violations")
    for violation in report.linting.violations:
        print(f"  {violation.file}:{violation.line} - {violation.message}")
```

### CLI Usage

```bash
# Run quality validation
python -m src.cli.main validate quality /path/to/project

# Get results
python -m src.cli.main results <run_id>
```

---

## Error Handling

### Error Response Format

```json
{
  "error": "VALIDATION_ERROR",
  "message": "Invalid repository path",
  "details": {
    "field": "repository",
    "reason": "Path does not exist"
  }
}
```

### Common Errors

| Error Code | HTTP Status | Description |
|------------|-------------|-------------|
| `VALIDATION_ERROR` | 400 | Invalid request parameters |
| `NOT_FOUND` | 404 | Resource not found |
| `SERVICE_UNAVAILABLE` | 503 | Too many concurrent validations |
| `TIMEOUT` | 504 | Validation timed out |

---

## Rate Limiting

- Maximum 10 concurrent validations per user
- Maximum 100 validations per hour per user
- Full validation pipeline limited to 15 minutes
- Individual quality checks limited to 5 minutes

---

## See Also

- [OpenAPI Specification](../../specs/001-testing/contracts/validation-api.yaml)
- [Data Models](../../src/core/models.py)
- [Quality Validator](../../src/quality/validator.py)
