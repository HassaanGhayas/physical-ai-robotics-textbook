# Configuration Guide

The Validation Service is highly configurable through environment variables and configuration files.

## Configuration Sources

Configuration is loaded in the following order (later sources override earlier):

1. Default values (built-in)
2. `.env` file
3. Environment variables
4. Command-line arguments

## Environment Variables

### General Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `VALIDATION_ENV` | Environment mode | `development` |
| `LOG_LEVEL` | Logging level | `INFO` |
| `LOG_FORMAT` | Log format (text/json) | `text` |

### API Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `HOST` | API host | `0.0.0.0` |
| `PORT` | API port | `8000` |
| `CORS_ORIGINS` | Allowed CORS origins | `*` |
| `API_DEBUG` | Enable debug mode | `false` |

### Quality Validation

| Variable | Description | Default |
|----------|-------------|---------|
| `ENABLE_QUALITY_VALIDATION` | Enable quality checks | `true` |
| `MIN_COVERAGE_PERCENTAGE` | Minimum coverage | `80` |
| `MAX_CYCLOMATIC_COMPLEXITY` | Max complexity | `10` |
| `MAX_LINE_LENGTH` | Max line length | `120` |
| `MAX_FUNCTION_LENGTH` | Max function lines | `50` |
| `ENABLE_SECURITY_SCAN` | Enable Bandit scan | `true` |

### Test Execution

| Variable | Description | Default |
|----------|-------------|---------|
| `ENABLE_TEST_EXECUTION` | Enable tests | `true` |
| `PARALLEL_EXECUTION` | Run tests in parallel | `true` |
| `MAX_PARALLEL_WORKERS` | Max parallel workers | `4` |
| `TEST_TIMEOUT` | Test timeout (seconds) | `300` |
| `FLAKY_DETECTION_ENABLED` | Detect flaky tests | `true` |
| `FLAKY_RETRY_COUNT` | Retries for flaky | `3` |

### Deployment Validation

| Variable | Description | Default |
|----------|-------------|---------|
| `ENABLE_DEPLOYMENT_VALIDATION` | Enable deployment checks | `true` |
| `REQUIRE_PASSING_TESTS` | Require tests to pass | `true` |
| `REQUIRE_PASSING_QUALITY` | Require quality to pass | `true` |
| `CHECK_VULNERABILITIES` | Check for vulnerabilities | `true` |

### Pipeline Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `PIPELINE_TIMEOUT_MINUTES` | Overall timeout | `15` |
| `FAIL_FAST` | Stop on first failure | `false` |

### Storage Settings

| Variable | Description | Default |
|----------|-------------|---------|
| `STORAGE_DATA_DIR` | Data directory | `.validation-data` |
| `STORAGE_RETENTION_DAYS` | Data retention | `30` |
| `MAX_HISTORY_ENTRIES` | Max history entries | `1000` |

## Example Configurations

### Development

```env
VALIDATION_ENV=development
LOG_LEVEL=DEBUG
MIN_COVERAGE_PERCENTAGE=60
FAIL_FAST=true
PARALLEL_EXECUTION=true
```

### CI/CD

```env
VALIDATION_ENV=ci
LOG_LEVEL=INFO
LOG_FORMAT=json
MIN_COVERAGE_PERCENTAGE=80
FAIL_FAST=true
TEST_TIMEOUT=600
FLAKY_RETRY_COUNT=2
```

### Production

```env
VALIDATION_ENV=production
LOG_LEVEL=WARNING
LOG_FORMAT=json
MIN_COVERAGE_PERCENTAGE=90
REQUIRE_PASSING_TESTS=true
REQUIRE_PASSING_QUALITY=true
CHECK_VULNERABILITIES=true
PIPELINE_TIMEOUT_MINUTES=30
```

## Configuration File

You can also use a `config.yaml` file:

```yaml
environment: development
log_level: INFO

quality:
  enabled: true
  min_coverage: 80
  max_complexity: 10
  max_line_length: 120

testing:
  enabled: true
  parallel: true
  max_workers: 4
  timeout: 300
  flaky_detection:
    enabled: true
    min_runs: 10
    pass_threshold: 0.8
    retry_count: 3

deployment:
  enabled: true
  require_tests: true
  require_quality: true
  check_vulnerabilities: true

pipeline:
  timeout_minutes: 15
  fail_fast: false
```

## Validation Rules

### Coverage Thresholds

| Environment | Recommended Coverage |
|-------------|---------------------|
| Development | 60% |
| Staging | 80% |
| Production | 90% |

### Complexity Limits

| Metric | Warning | Error |
|--------|---------|-------|
| Cyclomatic Complexity | 10 | 15 |
| Cognitive Complexity | 15 | 20 |
| Function Length | 50 | 100 |

### Flaky Test Detection

| Setting | Description | Recommended |
|---------|-------------|-------------|
| `min_runs` | Minimum runs before detection | 10 |
| `pass_threshold` | Threshold for flagging | 0.8 |
| `confidence_level` | Statistical confidence | 0.95 |

## Customizing Validators

### Custom Linting Rules

```yaml
quality:
  linting:
    rules:
      - E501  # Line too long
      - F401  # Unused import
    ignore:
      - W503  # Line break before binary operator
```

### Custom Test Patterns

```yaml
testing:
  patterns:
    unit: "tests/unit/**/*.py"
    integration: "tests/integration/**/*.py"
    e2e: "tests/e2e/**/*.py"
```

### Custom Deployment Checks

```yaml
deployment:
  checks:
    - name: "database_migrations"
      enabled: true
    - name: "env_variables"
      enabled: true
      required_vars:
        - DATABASE_URL
        - SECRET_KEY
```

## Runtime Configuration

Some settings can be overridden at runtime:

```python
from src.core.pipeline import PipelineConfig

config = PipelineConfig(
    run_quality=True,
    run_tests=True,
    run_deployment=False,
    fail_fast=True,
    timeout_minutes=10,
)
```

## Configuration Validation

Validate your configuration:

```bash
# Using CLI
validation-service config validate

# Using API
curl http://localhost:8000/api/v1/health?details=true
```
