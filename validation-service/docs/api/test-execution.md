# Test Execution API Documentation

This document describes the REST API endpoints for test execution functionality in the Validation Service.

## Overview

The Test Execution API provides endpoints for:
- Running test suites (unit, integration, E2E)
- Viewing test results and history
- Managing flaky tests
- Getting execution metrics

## Base URL

```
/api/v1
```

## Authentication

All endpoints require authentication via Bearer token:
```
Authorization: Bearer <token>
```

## Endpoints

### Run Tests

#### POST /validate/tests

Trigger test execution for a project.

**Request Body:**
```json
{
  "project_path": "/path/to/project",
  "repository": "owner/repo",
  "branch": "main",
  "commit_sha": "abc123def456",
  "suite_types": ["unit", "integration", "e2e"],
  "stop_on_failure": true,
  "parallel": true,
  "timeout_minutes": 15
}
```

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| project_path | string | Yes | Path to project directory |
| repository | string | Yes | Repository name |
| branch | string | Yes | Git branch name |
| commit_sha | string | Yes | Git commit SHA |
| suite_types | array | No | Test suites to run (default: all) |
| stop_on_failure | boolean | No | Stop on first failure (default: true) |
| parallel | boolean | No | Run tests in parallel (default: true) |
| timeout_minutes | integer | No | Timeout in minutes (default: 15) |

**Response:**
```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "running",
  "started_at": "2025-01-15T10:30:00Z",
  "estimated_completion": "2025-01-15T10:35:00Z"
}
```

**Status Codes:**
- `202 Accepted` - Test execution started
- `400 Bad Request` - Invalid request parameters
- `401 Unauthorized` - Invalid or missing authentication
- `409 Conflict` - Tests already running for this project

---

### Get Test Results

#### GET /results/{run_id}

Get full test results for a validation run.

**Path Parameters:**
- `run_id` - UUID of the validation run

**Response:**
```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "passed",
  "started_at": "2025-01-15T10:30:00Z",
  "completed_at": "2025-01-15T10:34:15Z",
  "duration_seconds": 255.0,
  "test_results": [
    {
      "suite_type": "unit",
      "status": "passed",
      "total_tests": 150,
      "passed_count": 148,
      "failed_count": 0,
      "skipped_count": 2,
      "error_count": 0,
      "duration_seconds": 45.5,
      "coverage_contribution": 85.5,
      "test_cases": [
        {
          "test_id": "tests.unit.test_auth::test_login",
          "test_name": "test_login",
          "status": "passed",
          "duration_seconds": 0.125,
          "error_message": null,
          "stack_trace": null,
          "retries": 0
        }
      ],
      "flaky_tests_detected": []
    },
    {
      "suite_type": "integration",
      "status": "passed",
      "total_tests": 45,
      "passed_count": 45,
      "failed_count": 0,
      "skipped_count": 0,
      "error_count": 0,
      "duration_seconds": 120.3
    },
    {
      "suite_type": "e2e",
      "status": "passed",
      "total_tests": 15,
      "passed_count": 15,
      "failed_count": 0,
      "skipped_count": 0,
      "error_count": 0,
      "duration_seconds": 89.2
    }
  ]
}
```

**Status Codes:**
- `200 OK` - Results returned
- `404 Not Found` - Run ID not found

---

### Get Test Status

#### GET /results/{run_id}/status

Get current status of a test run (lightweight).

**Path Parameters:**
- `run_id` - UUID of the validation run

**Response:**
```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "running",
  "current_stage": "integration_tests",
  "progress": {
    "completed_stages": ["unit_tests"],
    "current_stage": "integration_tests",
    "remaining_stages": ["e2e_tests"]
  },
  "elapsed_seconds": 125.5
}
```

---

### Get Test History

#### GET /history/tests/{test_id}

Get execution history for a specific test.

**Path Parameters:**
- `test_id` - Unique test identifier

**Query Parameters:**
- `limit` - Maximum history entries (default: 10, max: 100)

**Response:**
```json
{
  "test_id": "tests.unit.test_auth::test_login",
  "test_name": "test_login",
  "test_type": "unit",
  "file_path": "tests/unit/test_auth.py",
  "target_functionality": "auth",
  "execution_history": [
    {
      "run_id": "550e8400-e29b-41d4-a716-446655440000",
      "timestamp": "2025-01-15T10:30:00Z",
      "status": "passed",
      "duration_seconds": 0.125,
      "commit_sha": "abc123"
    },
    {
      "run_id": "550e8400-e29b-41d4-a716-446655440001",
      "timestamp": "2025-01-14T14:20:00Z",
      "status": "passed",
      "duration_seconds": 0.130,
      "commit_sha": "def456"
    }
  ],
  "average_duration_seconds": 0.127,
  "flakiness_score": 0.0,
  "is_flaky": false,
  "last_execution": "2025-01-15T10:30:00Z"
}
```

---

### List Flaky Tests

#### GET /flaky-tests

Get all detected flaky tests.

**Query Parameters:**
- `status` - Filter by status (`active_flaky`, `under_observation`, `resolved`)
- `min_confidence` - Minimum confidence threshold (0.0-1.0)
- `limit` - Maximum results (default: 50)

**Response:**
```json
{
  "flaky_tests": [
    {
      "test_id": "tests.e2e.test_checkout::test_payment_flow",
      "detected_at": "2025-01-10T08:15:00Z",
      "last_flake_at": "2025-01-15T09:45:00Z",
      "confidence": 0.95,
      "pass_rate": 0.78,
      "sample_size": 25,
      "consecutive_passes": 3,
      "status": "active_flaky",
      "failure_modes": [
        {
          "error_signature": "timeout_error",
          "frequency": 0.65,
          "sample_error": "TimeoutError: Page did not load within 30s"
        },
        {
          "error_signature": "assertion_error",
          "frequency": 0.35,
          "sample_error": "AssertionError: Expected element not found"
        }
      ]
    }
  ],
  "total_count": 1,
  "statistics": {
    "active_flaky": 1,
    "under_observation": 2,
    "resolved": 5
  }
}
```

---

### Update Flaky Test

#### PATCH /flaky-tests/{test_id}

Update the status of a flaky test.

**Path Parameters:**
- `test_id` - Test identifier

**Request Body:**
```json
{
  "status": "under_observation",
  "notes": "Added retry logic, monitoring for improvement"
}
```

**Response:**
```json
{
  "test_id": "tests.e2e.test_checkout::test_payment_flow",
  "status": "under_observation",
  "updated_at": "2025-01-15T11:00:00Z",
  "notes": "Added retry logic, monitoring for improvement"
}
```

**Status Codes:**
- `200 OK` - Test updated
- `404 Not Found` - Test not found

---

## Data Models

### TestSuiteResult

| Field | Type | Description |
|-------|------|-------------|
| run_id | UUID | Validation run ID |
| suite_type | string | Test suite type (unit/integration/e2e) |
| status | string | Result status (passed/failed/error/skipped) |
| started_at | datetime | Start timestamp |
| completed_at | datetime | End timestamp |
| duration_seconds | float | Execution duration |
| total_tests | integer | Total test count |
| passed_count | integer | Passed tests |
| failed_count | integer | Failed tests |
| skipped_count | integer | Skipped tests |
| error_count | integer | Tests with errors |
| test_cases | array | Individual test results |
| flaky_tests_detected | array | IDs of detected flaky tests |
| coverage_contribution | float | Coverage percentage |

### TestCaseResult

| Field | Type | Description |
|-------|------|-------------|
| test_id | string | Unique test identifier |
| test_name | string | Human-readable test name |
| status | string | Result status |
| duration_seconds | float | Execution duration |
| error_message | string | Error message (if failed) |
| stack_trace | string | Stack trace (if failed) |
| retries | integer | Number of retry attempts |

### FlakyTestRecord

| Field | Type | Description |
|-------|------|-------------|
| test_id | string | Unique test identifier |
| detected_at | datetime | When flakiness was detected |
| last_flake_at | datetime | Last failure timestamp |
| confidence | float | Confidence level (0-1) |
| pass_rate | float | Historical pass rate (0-1) |
| sample_size | integer | Number of executions analyzed |
| consecutive_passes | integer | Streak of consecutive passes |
| status | string | Flaky status |
| failure_modes | array | Distinct failure patterns |

---

## Error Responses

All error responses follow this format:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request parameters",
    "details": {
      "suite_types": "Invalid suite type 'unknown'"
    }
  }
}
```

### Error Codes

| Code | HTTP Status | Description |
|------|-------------|-------------|
| VALIDATION_ERROR | 400 | Invalid request parameters |
| UNAUTHORIZED | 401 | Authentication required |
| FORBIDDEN | 403 | Insufficient permissions |
| NOT_FOUND | 404 | Resource not found |
| CONFLICT | 409 | Resource conflict |
| TIMEOUT | 408 | Request timeout |
| INTERNAL_ERROR | 500 | Internal server error |

---

## Rate Limits

- Test execution: 10 requests per minute
- Results queries: 100 requests per minute
- Flaky test updates: 30 requests per minute

Rate limit headers:
```
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 95
X-RateLimit-Reset: 1642253400
```

---

## Webhooks

Configure webhooks to receive notifications:

### Test Completion

```json
{
  "event": "test.completed",
  "timestamp": "2025-01-15T10:34:15Z",
  "data": {
    "run_id": "550e8400-e29b-41d4-a716-446655440000",
    "status": "passed",
    "repository": "owner/repo",
    "branch": "main",
    "commit_sha": "abc123",
    "summary": {
      "total_tests": 210,
      "passed": 208,
      "failed": 0,
      "skipped": 2,
      "duration_seconds": 255.0
    }
  }
}
```

### Flaky Test Detected

```json
{
  "event": "flaky_test.detected",
  "timestamp": "2025-01-15T09:45:00Z",
  "data": {
    "test_id": "tests.e2e.test_checkout::test_payment_flow",
    "confidence": 0.95,
    "pass_rate": 0.78,
    "repository": "owner/repo"
  }
}
```
