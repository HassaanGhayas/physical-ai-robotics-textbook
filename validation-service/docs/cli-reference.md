# CLI Reference

The Validation Service CLI provides commands for running validations and managing results from the command line.

## Installation

After installing the validation service package, the `validation-service` command is available:

```bash
pip install -e .
validation-service --help
```

## Commands

### run

Run validation pipeline on a project.

```bash
validation-service run [OPTIONS]
```

**Options:**

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--path` | `-p` | Path to project to validate | `.` |
| `--quality/--no-quality` | | Run quality checks | `--quality` |
| `--tests/--no-tests` | | Run test suites | `--tests` |
| `--deployment/--no-deployment` | | Run deployment checks | `--deployment` |
| `--fail-fast` | | Stop on first failure | `false` |
| `--verbose` | `-v` | Enable verbose output | `false` |

**Examples:**

```bash
# Run full validation on current directory
validation-service run

# Run on a specific project
validation-service run --path /path/to/project

# Run quality checks only
validation-service run --no-tests --no-deployment

# Run tests only with verbose output
validation-service run --no-quality --no-deployment --verbose

# Run with fail-fast
validation-service run --fail-fast
```

### results show

Display validation results for a specific run.

```bash
validation-service results show RUN_ID [OPTIONS]
```

**Arguments:**

| Argument | Description |
|----------|-------------|
| `RUN_ID` | The validation run ID (UUID) |

**Options:**

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--format` | `-f` | Output format (table, json) | `table` |
| `--verbose` | `-v` | Show detailed output | `false` |

**Examples:**

```bash
# Show results in table format
validation-service results show 550e8400-e29b-41d4-a716-446655440000

# Show results as JSON
validation-service results show 550e8400-e29b-41d4-a716-446655440000 --format json

# Show verbose results
validation-service results show 550e8400-e29b-41d4-a716-446655440000 --verbose
```

### results latest

Display the most recent validation results.

```bash
validation-service results latest [OPTIONS]
```

**Options:**

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--format` | `-f` | Output format (table, json) | `table` |

**Examples:**

```bash
# Show latest results
validation-service results latest

# Show latest results as JSON
validation-service results latest --format json
```

### flaky-tests list

List detected flaky tests.

```bash
validation-service flaky-tests list [OPTIONS]
```

**Options:**

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--status` | `-s` | Filter by status | `all` |
| `--limit` | `-l` | Maximum results | `20` |

**Examples:**

```bash
# List all flaky tests
validation-service flaky-tests list

# List active flaky tests only
validation-service flaky-tests list --status active_flaky

# List with limit
validation-service flaky-tests list --limit 10
```

### flaky-tests update

Update the status of a flaky test.

```bash
validation-service flaky-tests update TEST_ID --status STATUS
```

**Arguments:**

| Argument | Description |
|----------|-------------|
| `TEST_ID` | The test identifier |

**Options:**

| Option | Short | Description | Required |
|--------|-------|-------------|----------|
| `--status` | `-s` | New status | Yes |

**Valid Statuses:**

- `active_flaky` - Test is actively flaky
- `resolved` - Test has been fixed
- `quarantined` - Test is quarantined
- `acknowledged` - Test is acknowledged but not fixed

**Examples:**

```bash
# Mark a flaky test as resolved
validation-service flaky-tests update "test_module::test_function" --status resolved

# Quarantine a flaky test
validation-service flaky-tests update "test_module::test_function" --status quarantined
```

### config show

Display current configuration.

```bash
validation-service config show
```

**Example:**

```bash
validation-service config show
```

**Output:**

```
┌─────────────────────────────────────────┐
│        Current Configuration            │
├──────────────────┬──────────────────────┤
│ Setting          │ Value                │
├──────────────────┼──────────────────────┤
│ Environment      │ development          │
│ Log Level        │ INFO                 │
│ Quality Enabled  │ True                 │
│ Tests Enabled    │ True                 │
│ Deployment Enabled│ True                │
│ Pipeline Timeout │ 15 min               │
│ Min Coverage     │ 80%                  │
│ Max Complexity   │ 10                   │
│ Parallel Tests   │ True                 │
│ Flaky Detection  │ True                 │
└──────────────────┴──────────────────────┘
```

### config validate

Validate the current configuration.

```bash
validation-service config validate
```

**Example:**

```bash
validation-service config validate
```

### status

Show service status and recent activity.

```bash
validation-service status
```

**Example:**

```bash
validation-service status
```

**Output:**

```
┌───────────────────────────────────────┐
│     Validation Service Status         │
├───────────────────────────────────────┤
│ Environment      development          │
│ Quality Checks   Enabled              │
│ Test Execution   Enabled              │
│ Deployment Checks Enabled             │
└───────────────────────────────────────┘

Recent Validation Runs
┌──────────┬─────────┬────────────┬────────┐
│ Run ID   │ Status  │ Repository │ Branch │
├──────────┼─────────┼────────────┼────────┤
│ abc123.. │ PASSED  │ my-repo    │ main   │
└──────────┴─────────┴────────────┴────────┘

Flaky Tests Summary
  Active: 2
  Resolved: 5
```

## Exit Codes

| Code | Description |
|------|-------------|
| 0 | Success / Validation passed |
| 1 | Failure / Validation failed |
| 2 | Error / Invalid arguments |

## Environment Variables

The CLI respects the following environment variables:

| Variable | Description |
|----------|-------------|
| `VALIDATION_ENV` | Environment (development/production) |
| `LOG_LEVEL` | Logging level |
| `NO_COLOR` | Disable colored output |

## Tips

### Running in CI/CD

```bash
# Exit with appropriate code based on validation result
validation-service run --path . || exit 1
```

### JSON Output for Scripting

```bash
# Get results as JSON for parsing
validation-service results latest --format json | jq '.status'
```

### Verbose Debugging

```bash
# Enable debug logging
LOG_LEVEL=DEBUG validation-service run --verbose
```
