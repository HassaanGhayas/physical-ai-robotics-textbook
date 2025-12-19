# Quick Start Guide: Testing System

**Feature**: Thorough Checking & Testing System
**Date**: 2025-12-17
**Audience**: Developers onboarding to the validation system

## Overview

This guide helps you set up and run the automated validation system locally within 15 minutes. You'll learn how to trigger validations, interpret results, and integrate with your development workflow.

---

## Prerequisites

**Required**:
- Python 3.11+ installed
- Node.js 18+ (for TypeScript/JavaScript projects)
- Git repository with existing test suite
- Terminal access (bash/PowerShell/cmd)

**Optional but Recommended**:
- CI/CD platform access (GitHub Actions, GitLab CI, or Jenkins)
- Text editor with JSON/YAML support

---

## Installation

### Step 1: Clone or Navigate to Repository

```bash
cd /path/to/your/repository
```

### Step 2: Install Validation Service

```bash
# Install from repository root
./scripts/install.sh

# Or manually:
pip install -r validation-service/requirements.txt
npm install -g validation-cli  # For Node.js test integration
```

### Step 3: Verify Installation

```bash
validate --version
# Expected output: Validation Service v1.0.0
```

---

## First Validation Run

### Local Validation (Quick Check)

Run validation on your current code before committing:

```bash
# Full validation (quality + tests + deployment)
validate run

# Specific checks only
validate run --quality          # Code quality only
validate run --tests unit       # Unit tests only
validate run --deployment       # Deployment checks only
```

**Expected Output**:
```
✓ Code Quality: PASSED (2.3s)
  - Linting: 0 violations
  - Complexity: Average 4.2 (max 8)
  - Coverage: 78% (target: 60%)

✓ Unit Tests: PASSED (5.1s)
  - 127/127 tests passed
  - 0 flaky tests detected

✓ Integration Tests: PASSED (8.4s)
  - 45/45 tests passed

⚠ E2E Tests: WARNING (12.7s)
  - 18/19 tests passed
  - 1 flaky test detected: test_login_retry

✓ Deployment Checks: PASSED (1.9s)
  - Configuration: valid
  - Dependencies: 0 conflicts, 0 vulnerabilities
  - Compatibility: passed

Overall Status: PASSED (30.4s)
Run ID: a1b2c3d4-e5f6-7890-abcd-ef1234567890
```

### View Detailed Results

```bash
# Open HTML report in browser
validate results a1b2c3d4-e5f6-7890-abcd-ef1234567890 --format html

# View JSON for programmatic access
validate results a1b2c3d4-e5f6-7890-abcd-ef1234567890 --format json

# Check specific section
validate results a1b2c3d4 --section quality
```

---

## Understanding Results

### Pass/Fail Criteria

| Check Type | Pass Criteria |
|------------|---------------|
| **Linting** | 0 error-level violations |
| **Complexity** | No functions >10 cyclomatic complexity |
| **Coverage** | ≥60% overall, ≥80% for critical paths |
| **Unit Tests** | 100% pass rate, <5% flaky |
| **Integration** | 100% pass rate, <5% flaky |
| **E2E Tests** | ≥95% pass rate |
| **Config** | All required env vars present |
| **Dependencies** | 0 critical vulnerabilities |

### Status Codes

- **PASSED** ✓: All checks met criteria
- **WARNING** ⚠: Non-critical issues (e.g., flaky tests, outdated packages)
- **FAILED** ✗: Critical issues block progression
- **ERROR** ⚠️: System error (tool crash, timeout)

---

## Flaky Test Detection

### Identifying Flaky Tests

The system automatically detects tests with inconsistent pass/fail patterns:

```bash
# List all flaky tests
validate flaky-tests list

# Output:
# ACTIVE FLAKY TESTS (2)
# - test_login_retry (pass rate: 75%, confidence: 97%)
# - test_cache_expiry (pass rate: 82%, confidence: 93%)
```

### Understanding Flaky Test Data

```bash
validate flaky-tests show test_login_retry
```

**Output**:
```json
{
  "test_id": "tests/auth/test_login.py::test_login_retry",
  "detected_at": "2025-12-15T10:30:00Z",
  "pass_rate": 0.75,
  "confidence": 0.97,
  "execution_pattern": [true, true, false, true, true, false, true, false, true, true],
  "failure_modes": [
    {
      "error_signature": "TimeoutError: Connection pool timeout",
      "frequency": 0.67,
      "sample_error": "requests.exceptions.Timeout: HTTPConnectionPool..."
    }
  ],
  "consecutive_passes": 2
}
```

**Key Metrics**:
- **Pass Rate**: Proportion of passing executions (75% = fails 1 in 4 times)
- **Confidence**: Statistical confidence in flakiness (97% = very confident)
- **Execution Pattern**: Last 10 runs (true=pass, false=fail)
- **Failure Modes**: Distinct ways the test fails

### Resolving Flaky Tests

1. **Investigate Root Cause**: Check `failure_modes` for error patterns
2. **Fix Test or Code**: Address timing issues, race conditions, external dependencies
3. **Mark as Resolved**: After 5 consecutive passes, test auto-clears from flaky list
4. **Manual Override**: Mark as false positive if detection was incorrect

```bash
# Mark as false positive
validate flaky-tests update test_login_retry --status false_positive --note "Transient CI issue"
```

---

## Integration with Development Workflow

### Pre-Commit Hook (Local Validation)

Add to `.git/hooks/pre-commit`:

```bash
#!/bin/bash
# Run fast local validation before commit
validate run --quality --tests unit --timeout 300

if [ $? -ne 0 ]; then
    echo "❌ Validation failed. Fix issues before committing."
    exit 1
fi

echo "✓ Local validation passed"
exit 0
```

### CI/CD Integration

#### GitHub Actions

Create `.github/workflows/validation.yml`:

```yaml
name: Validation

on: [push, pull_request]

jobs:
  validate:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'
      - name: Install validation service
        run: ./scripts/install.sh
      - name: Run validation
        run: validate run --ci-mode
      - name: Upload results
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: validation-results
          path: .validation/results/
```

#### GitLab CI

Add to `.gitlab-ci.yml`:

```yaml
validation:
  stage: test
  image: python:3.11
  script:
    - ./scripts/install.sh
    - validate run --ci-mode
  artifacts:
    reports:
      junit: .validation/results/junit.xml
    paths:
      - .validation/results/
    when: always
```

---

## Configuration

### Validation Config File

Create `.validation/config.yaml` to customize behavior:

```yaml
# Quality thresholds
quality:
  coverage:
    overall_target: 60
    critical_path_target: 80
  complexity:
    max_cyclomatic: 10
    warning_threshold: 8
  linting:
    fail_on_errors: true
    fail_on_warnings: false

# Test execution
tests:
  unit:
    timeout_seconds: 300
    parallel: true
    max_workers: 4
  integration:
    timeout_seconds: 600
    parallel: false
  e2e:
    timeout_seconds: 900
    retry_on_failure: true
    max_retries: 2

# Flaky test detection
flaky_tests:
  confidence_threshold: 0.95
  pass_rate_threshold: 0.80
  sample_size: 10
  auto_resolve_after: 5  # consecutive passes

# Deployment checks
deployment:
  required_env_vars:
    - DATABASE_URL
    - API_KEY
    - SECRET_KEY
  check_vulnerabilities: true
  fail_on_outdated: false

# Reporting
reporting:
  formats: [junit, json, html]
  output_dir: .validation/results
```

### Environment Variables

```bash
# Override config via environment variables
export VALIDATION_COVERAGE_TARGET=70
export VALIDATION_TIMEOUT=900
export VALIDATION_PARALLEL_TESTS=true
```

---

## Common Workflows

### Before Committing Code

```bash
# Quick local check (<5 min)
validate run --fast

# Full local validation
validate run
```

### Before Creating Pull Request

```bash
# Comprehensive validation
validate run --full

# Check against main branch
git fetch origin main
validate run --compare origin/main
```

### Investigating Test Failures

```bash
# Get detailed failure info
validate results <run_id> --section tests --verbose

# View specific test logs
validate logs <run_id> --test "test_name"

# Check test execution history
validate history tests/<path>/test_name.py::test_function
```

### Monitoring Flaky Tests

```bash
# Weekly flaky test review
validate flaky-tests list --since 7d --format table

# Export for team review
validate flaky-tests list --format csv > flaky-tests-report.csv
```

---

## Troubleshooting

### Validation Times Out

```bash
# Increase timeout
validate run --timeout 1800  # 30 minutes

# Run phases separately
validate run --quality
validate run --tests unit
validate run --tests integration
validate run --deployment
```

### High False Positive Rate for Flaky Tests

```bash
# Adjust detection sensitivity
validate config set flaky_tests.confidence_threshold 0.98
validate config set flaky_tests.sample_size 15
```

### Coverage Not Calculated

```bash
# Verify coverage tools installed
pip show coverage
npm list -g istanbul

# Run tests with coverage explicitly
pytest --cov=src tests/
jest --coverage
```

### CI/CD Integration Issues

```bash
# Test CI mode locally
validate run --ci-mode --verbose

# Check logs
cat .validation/logs/latest.log
```

---

## Best Practices

1. **Run Locally First**: Always validate locally before pushing to CI/CD
2. **Fix Flaky Tests Promptly**: Don't let flaky tests accumulate (target: <5% of test suite)
3. **Monitor Validation Time**: Keep pipeline under 15 minutes for fast feedback
4. **Review Quality Trends**: Track coverage and complexity over time
5. **Configure Per Project**: Customize thresholds based on project needs
6. **Update Dependencies**: Keep validation tools up to date

---

## Getting Help

```bash
# Show help for any command
validate --help
validate run --help

# View validation service logs
validate logs --tail 100

# Check service status
validate status
```

**Support**:
- Documentation: `./docs/validation-system/`
- Issues: Open GitHub issue with `[validation]` tag
- Configuration examples: `./validation-service/examples/`

---

## Next Steps

- **Set up CI/CD integration**: Add validation to your pipeline
- **Configure thresholds**: Customize quality gates for your project
- **Enable pre-commit hooks**: Catch issues before committing
- **Monitor flaky tests**: Regular review and resolution
- **Explore advanced features**: Parallel execution, custom reports, webhooks

For detailed API documentation, see `contracts/validation-api.yaml`.
For complete data models, see `data-model.md`.
