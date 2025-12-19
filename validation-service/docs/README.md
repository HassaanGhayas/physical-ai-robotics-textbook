# Validation Service Documentation

Welcome to the Validation Service documentation. This service provides comprehensive code quality, testing, and deployment validation for your projects.

## Quick Links

- [Getting Started](./getting-started.md) - Installation and basic usage
- [API Reference](./api-reference.md) - REST API documentation
- [CLI Reference](./cli-reference.md) - Command-line interface
- [Configuration](./configuration.md) - Configuration options
- [Deployment](./deployment.md) - Deployment guide

## Overview

The Validation Service is a comprehensive tool that ensures code quality, test coverage, and deployment readiness for your projects. It integrates seamlessly into CI/CD pipelines and provides detailed feedback.

### Key Features

- **Code Quality Validation**
  - Coverage analysis
  - Linting (Ruff, Flake8)
  - Complexity analysis
  - Security scanning (Bandit)

- **Test Execution**
  - Unit, integration, and E2E test support
  - Parallel test execution
  - Flaky test detection
  - Test history tracking

- **Deployment Validation**
  - Configuration validation
  - Dependency checking
  - Security scanning
  - Compatibility verification

## Architecture

```
validation-service/
├── src/
│   ├── api/              # FastAPI REST endpoints
│   ├── cli/              # Typer CLI commands
│   ├── core/             # Core models, config, pipeline
│   ├── quality/          # Quality validation modules
│   ├── testing/          # Test execution modules
│   └── deployment/       # Deployment validation modules
├── tests/                # Test suite
├── docs/                 # Documentation
└── scripts/              # Utility scripts
```

## Usage

### API

```bash
# Start the API server
uvicorn src.api.main:app --reload

# Run validation via API
curl -X POST http://localhost:8000/api/v1/validate \
  -H "Content-Type: application/json" \
  -d '{"project_path": "/path/to/project", "repository": "my-repo", "branch": "main", "commit_sha": "abc123"}'
```

### CLI

```bash
# Run full validation
validation-service run --path /path/to/project

# Quality-only validation
validation-service run --path /path/to/project --no-tests --no-deployment

# View results
validation-service results show <run-id>

# Check service status
validation-service status
```

### Docker

```bash
# Build and run
docker-compose up -d validation-service

# Run validation
docker-compose exec validation-service python -m src.cli run --path /app
```

## Support

For issues and feature requests, please open a GitHub issue.

## License

MIT License - see LICENSE file for details.
