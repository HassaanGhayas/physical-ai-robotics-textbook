# Deployment Validation API Documentation

This document describes the REST API endpoints for deployment validation functionality in the Validation Service.

## Overview

The Deployment Validation API provides endpoints for:
- Validating deployment configurations
- Checking dependencies for conflicts and vulnerabilities
- Scanning for security issues
- Verifying environment compatibility

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

### Run Deployment Validation

#### POST /validate/deployment

Trigger deployment validation for a project.

**Request Body:**
```json
{
  "project_path": "/path/to/project",
  "repository": "owner/repo",
  "branch": "main",
  "commit_sha": "abc123def456",
  "checks": {
    "configuration": true,
    "dependencies": true,
    "security": true,
    "compatibility": true
  },
  "target_environment": "production"
}
```

**Parameters:**

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| project_path | string | Yes | Path to project directory |
| repository | string | Yes | Repository name |
| branch | string | Yes | Git branch name |
| commit_sha | string | Yes | Git commit SHA |
| checks | object | No | Which checks to run (default: all) |
| target_environment | string | No | Target environment (dev/staging/production) |

**Response:**
```json
{
  "run_id": "550e8400-e29b-41d4-a716-446655440000",
  "status": "passed",
  "is_deployable": true,
  "validated_at": "2025-01-15T10:30:00Z",
  "configuration": {
    "status": "passed",
    "required_variables": 10,
    "missing_variables": 0,
    "invalid_values": 0,
    "secret_exposures": 0
  },
  "dependencies": {
    "status": "passed",
    "total_packages": 25,
    "vulnerable_packages": 0,
    "outdated_packages": 3,
    "conflicts": []
  },
  "compatibility": {
    "status": "passed",
    "resource_checks": [],
    "platform_checks": [],
    "service_checks": []
  }
}
```

**Status Codes:**
- `200 OK` - Validation completed
- `400 Bad Request` - Invalid request parameters
- `401 Unauthorized` - Invalid or missing authentication
- `500 Internal Server Error` - Validation failed to execute

---

### Get Configuration Check Details

#### GET /validate/deployment/configuration/{run_id}

Get detailed configuration validation results.

**Path Parameters:**
- `run_id` - UUID of the validation run

**Response:**
```json
{
  "status": "warning",
  "checks": [
    {
      "name": "DATABASE_URL",
      "status": "valid",
      "is_sensitive": true,
      "actual_value": "***",
      "location": ".env"
    },
    {
      "name": "API_KEY",
      "status": "missing",
      "is_sensitive": true,
      "error_message": "Required variable has empty value"
    }
  ],
  "missing_required": ["API_KEY"],
  "invalid_values": [],
  "secret_risks": [
    {
      "file_path": "config.py",
      "line_number": 15,
      "secret_type": "API Key",
      "severity": "high",
      "pattern_matched": "api_key pattern",
      "recommendation": "Move API Key to environment variable"
    }
  ]
}
```

---

### Get Dependency Check Details

#### GET /validate/deployment/dependencies/{run_id}

Get detailed dependency validation results.

**Path Parameters:**
- `run_id` - UUID of the validation run

**Response:**
```json
{
  "status": "warning",
  "total_dependencies": 25,
  "security_vulnerabilities": 2,
  "conflicts": [
    {
      "package": "werkzeug",
      "required_by": ["flask"],
      "versions_required": [">=3.0.0"],
      "conflict_type": "version_conflict",
      "severity": "medium",
      "resolution": "Upgrade werkzeug to >=3.0.0"
    }
  ],
  "outdated_packages": [
    {
      "name": "requests",
      "current_version": "2.28.0",
      "latest_version": "2.31.0",
      "latest_stable": "2.31.0",
      "severity": "low",
      "has_security_update": false
    }
  ],
  "vulnerabilities": [
    {
      "package": "cryptography",
      "current_version": "40.0.0",
      "vulnerability_id": "CVE-2023-XXXX",
      "severity": "high",
      "fixed_version": "41.0.0",
      "description": "Buffer overflow vulnerability"
    }
  ]
}
```

---

### Get Compatibility Check Details

#### GET /validate/deployment/compatibility/{run_id}

Get detailed compatibility validation results.

**Path Parameters:**
- `run_id` - UUID of the validation run

**Response:**
```json
{
  "status": "passed",
  "resource_checks": [
    {
      "resource_type": "disk_space",
      "required": "1GB",
      "available": "50.5GB",
      "status": "valid",
      "message": "Sufficient disk space"
    },
    {
      "resource_type": "memory",
      "required": "512MB",
      "available": "8.0GB",
      "status": "valid"
    },
    {
      "resource_type": "tool:python",
      "required": "installed",
      "available": "installed",
      "status": "valid"
    }
  ],
  "platform_checks": [
    {
      "platform": "python",
      "required_version": "3.10",
      "detected_version": "3.11.5",
      "is_compatible": true,
      "warnings": []
    },
    {
      "platform": "node",
      "required_version": "18.0.0",
      "detected_version": "20.10.0",
      "is_compatible": true,
      "warnings": []
    }
  ],
  "service_checks": [
    {
      "service_name": "database:postgres",
      "endpoint": "localhost:5432",
      "status": "valid",
      "latency_ms": 2.5,
      "is_required": true
    },
    {
      "service_name": "cache:redis",
      "endpoint": "localhost:6379",
      "status": "invalid",
      "error_message": "Connection refused",
      "is_required": false
    }
  ]
}
```

---

## Data Models

### DeploymentChecklist

| Field | Type | Description |
|-------|------|-------------|
| run_id | UUID | Validation run ID |
| validated_at | datetime | Validation timestamp |
| status | string | Overall status (passed/warning/failed) |
| is_deployable | boolean | Whether deployment can proceed |
| configuration | object | Configuration check results |
| dependencies | object | Dependency check results |
| compatibility | object | Compatibility check results |
| blocking_issues | array | Issues that block deployment |
| warnings | array | Non-blocking warnings |

### ConfigCheck

| Field | Type | Description |
|-------|------|-------------|
| name | string | Configuration item name |
| status | string | Check status |
| expected_type | string | Expected type |
| actual_value | string | Actual value (redacted if sensitive) |
| is_sensitive | boolean | Whether value is sensitive |
| error_message | string | Error message if invalid |
| location | string | File or location of config |

### SecretRisk

| Field | Type | Description |
|-------|------|-------------|
| file_path | string | File containing the potential secret |
| line_number | integer | Line number where detected |
| secret_type | string | Type of secret (API key, password, etc.) |
| severity | string | Severity level (critical/high/medium/low) |
| pattern_matched | string | Pattern that matched |
| recommendation | string | Recommended action |

### DependencyConflict

| Field | Type | Description |
|-------|------|-------------|
| package | string | Package name |
| required_by | array | Packages requiring this |
| versions_required | array | Version requirements |
| conflict_type | string | Type of conflict |
| severity | string | Severity level |
| resolution | string | Suggested resolution |

### OutdatedPackage

| Field | Type | Description |
|-------|------|-------------|
| name | string | Package name |
| current_version | string | Currently installed version |
| latest_version | string | Latest available version |
| latest_stable | string | Latest stable version |
| severity | string | Severity level |
| has_security_update | boolean | Whether update includes security fixes |
| changelog_url | string | URL to changelog |

### ResourceCheck

| Field | Type | Description |
|-------|------|-------------|
| resource_type | string | Type of resource (memory, cpu, disk, tool) |
| required | string | Required amount |
| available | string | Available amount |
| status | string | Check status |
| message | string | Additional message |

### PlatformCheck

| Field | Type | Description |
|-------|------|-------------|
| platform | string | Platform name (python, node, os) |
| required_version | string | Required version |
| detected_version | string | Detected version |
| is_compatible | boolean | Whether compatible |
| warnings | array | Warning messages |

### ServiceCheck

| Field | Type | Description |
|-------|------|-------------|
| service_name | string | Service name |
| endpoint | string | Service endpoint |
| status | string | Check status |
| latency_ms | float | Response latency |
| error_message | string | Error message if failed |
| is_required | boolean | Whether service is required |

---

## Error Responses

All error responses follow this format:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "Invalid request parameters",
    "details": {
      "project_path": "Path does not exist"
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
| INTERNAL_ERROR | 500 | Internal server error |

---

## Security Scanning Severity Levels

| Level | Description | Action |
|-------|-------------|--------|
| CRITICAL | Exploitable vulnerability | Must fix before deployment |
| HIGH | Significant security risk | Should fix before deployment |
| MEDIUM | Potential security concern | Review and consider fixing |
| LOW | Minor security issue | Fix when convenient |
| INFO | Informational finding | No action required |

---

## Webhooks

Configure webhooks to receive notifications:

### Deployment Validation Complete

```json
{
  "event": "deployment.validated",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "run_id": "550e8400-e29b-41d4-a716-446655440000",
    "status": "warning",
    "is_deployable": true,
    "repository": "owner/repo",
    "branch": "main",
    "summary": {
      "configuration": "passed",
      "dependencies": "warning",
      "compatibility": "passed"
    },
    "blocking_issues": [],
    "warnings": ["3 outdated packages found"]
  }
}
```

### Security Issue Detected

```json
{
  "event": "security.issue_detected",
  "timestamp": "2025-01-15T10:30:00Z",
  "data": {
    "run_id": "550e8400-e29b-41d4-a716-446655440000",
    "severity": "high",
    "issue_type": "hardcoded_secret",
    "file_path": "config.py",
    "line_number": 15,
    "description": "API Key detected in source code",
    "repository": "owner/repo"
  }
}
```
