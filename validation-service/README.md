# Validation Service

Automated code quality, testing, and deployment validation service.

## Overview

The Validation Service provides a comprehensive automated validation pipeline for software projects, covering:

- **Code Quality Validation** (P1): Linting, complexity analysis, coverage checking, security scanning
- **Test Suite Execution** (P2): Unit, integration, and E2E test orchestration with flaky test detection
- **Deployment Validation** (P3): Configuration validation, dependency checking, compatibility verification

## Quick Start

### Installation

```bash
# Clone the repository
cd validation-service

# Create virtual environment
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# For development
pip install -r requirements-dev.txt
```

### Configuration

Copy the example environment file:

```bash
cp .env.example .env
```

Edit `.env` to customize settings.

### Running Validation

#### CLI Usage

```bash
# Run full validation pipeline
validate run --path /path/to/project

# Run only code quality checks
validate quality --path /path/to/project

# Run only tests
validate tests --path /path/to/project

# Run only deployment checks
validate deployment --path /path/to/project

# Check validation results
validate results <run-id>

# List flaky tests
validate flaky-tests
```

#### API Usage

Start the API server:

```bash
uvicorn src.api.main:app --host 0.0.0.0 --port 8080
```

API endpoints:

- `POST /validate` - Run full validation
- `POST /validate/quality` - Run quality checks
- `POST /validate/tests` - Run test suite
- `POST /validate/deployment` - Run deployment checks
- `GET /results/{run_id}` - Get validation results
- `GET /flaky-tests` - List detected flaky tests

## Project Structure

```
validation-service/
├── src/
│   ├── core/           # Core models, pipeline, storage
│   ├── quality/        # Code quality validation
│   ├── testing/        # Test execution
│   ├── deployment/     # Deployment validation
│   ├── api/            # REST API
│   └── cli/            # Command-line interface
├── tests/
│   ├── unit/           # Unit tests
│   ├── integration/    # Integration tests
│   └── e2e/            # End-to-end tests
├── docs/               # Documentation
└── scripts/            # Utility scripts
```

## Configuration Options

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `LOG_LEVEL` | `INFO` | Logging level |
| `QUALITY_MAX_LINE_LENGTH` | `100` | Max line length for linting |
| `QUALITY_MIN_COVERAGE_PERCENTAGE` | `80.0` | Minimum test coverage |
| `TESTING_PARALLEL_EXECUTION` | `true` | Enable parallel tests |
| `TESTING_FLAKY_PASS_THRESHOLD` | `0.8` | Flaky test pass threshold |
| `DEPLOYMENT_CHECK_VULNERABILITIES` | `true` | Enable vulnerability scanning |

See `src/core/config.py` for all configuration options.

## Development

### Running Tests

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=src --cov-report=html

# Run specific test category
pytest -m unit
pytest -m integration
```

### Code Quality

```bash
# Run linter
ruff check src tests

# Run type checker
mypy src

# Run security scanner
bandit -r src
```

## License

MIT License
