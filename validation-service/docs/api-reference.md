# API Reference

The Validation Service provides a RESTful API for running validations and retrieving results.

## Base URL

```
http://localhost:8000/api/v1
```

## Authentication

Currently, the API does not require authentication. In production, configure authentication as needed.

## Endpoints

### Validation

#### Run Full Validation

```http
POST /validate
```

**Request Body:**

```json
{
  "project_path": "/path/to/project",
  "repository": "my-repo",
  "branch": "main",
  "commit_sha": "abc123def456",
  "run_quality": true,
  "run_tests": true,
  "run_deployment": true,
  "fail_fast": false,
  "timeout_minutes": 15
}
```

**Response:**

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "passed",
  "message": "Validation passed"
}
```

#### Run Quality Validation Only

```http
POST /validate/quality
```

#### Run Tests Only

```http
POST /validate/tests
```

#### Run Deployment Validation Only

```http
POST /validate/deployment
```

### Results

#### Get Full Results

```http
GET /results/{run_id}
```

**Response:**

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "passed",
  "trigger_source": "api",
  "repository": "my-repo",
  "branch": "main",
  "commit_sha": "abc123def456",
  "started_at": "2024-01-01T10:00:00Z",
  "completed_at": "2024-01-01T10:05:00Z",
  "duration_seconds": 300.0,
  "quality_report": {
    "coverage_percentage": 85.0,
    "passed": true,
    "issues": []
  },
  "test_results": [
    {
      "suite_type": "unit",
      "total_tests": 100,
      "passed_tests": 100,
      "failed_tests": 0,
      "skipped_tests": 0,
      "duration_seconds": 30.0,
      "passed": true,
      "failures": []
    }
  ],
  "deployment_checklist": {
    "passed": true,
    "checks": []
  }
}
```

#### Get Validation Status

```http
GET /results/{run_id}/status
```

**Response:**

```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "passed",
  "progress": {
    "completed_stages": ["quality", "unit_tests", "integration_tests", "deployment"],
    "current_stage": null
  },
  "elapsed_seconds": 300.0
}
```

### History

#### List Validation Runs

```http
GET /history
```

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `repository` | string | Filter by repository |
| `branch` | string | Filter by branch |
| `status` | string | Filter by status (passed, failed) |
| `limit` | integer | Maximum results (default: 20) |

**Response:**

```json
{
  "runs": [
    {
      "run_id": "550e8400-e29b-41d4-a716-446655440000",
      "status": "passed",
      "repository": "my-repo",
      "branch": "main",
      "started_at": "2024-01-01T10:00:00Z"
    }
  ]
}
```

#### Get Statistics

```http
GET /history/stats
```

**Response:**

```json
{
  "total_runs": 100,
  "pass_rate": 95.0,
  "average_duration": 45.5,
  "runs_by_status": {
    "passed": 95,
    "failed": 5
  }
}
```

#### Compare Runs

```http
GET /history/compare?run_id_1={id1}&run_id_2={id2}
```

### Flaky Tests

#### List Flaky Tests

```http
GET /flaky-tests
```

**Query Parameters:**

| Parameter | Type | Description |
|-----------|------|-------------|
| `status` | string | Filter by status |
| `limit` | integer | Maximum results (default: 20) |

**Response:**

```json
{
  "tests": [
    {
      "test_id": "test_module::test_function",
      "pass_rate": 0.75,
      "sample_size": 20,
      "confidence": 0.92,
      "status": "active_flaky"
    }
  ]
}
```

#### Update Flaky Test Status

```http
PATCH /flaky-tests/{test_id}
```

**Request Body:**

```json
{
  "status": "resolved"
}
```

#### Get Flaky Test Statistics

```http
GET /flaky-tests/stats
```

### Health

#### Health Check

```http
GET /health
```

**Response:**

```json
{
  "status": "healthy"
}
```

#### Readiness Check

```http
GET /health/ready
```

#### Liveness Check

```http
GET /health/live
```

#### Metrics

```http
GET /health/metrics
```

## Error Responses

All errors follow this format:

```json
{
  "detail": "Error message",
  "error_code": "ERROR_CODE",
  "status_code": 400
}
```

### Status Codes

| Code | Description |
|------|-------------|
| 200 | Success |
| 400 | Bad Request |
| 404 | Not Found |
| 422 | Validation Error |
| 500 | Internal Server Error |

## OpenAPI Documentation

Interactive API documentation is available at:

- Swagger UI: `http://localhost:8000/docs`
- ReDoc: `http://localhost:8000/redoc`
