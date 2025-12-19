# Getting Started

This guide will help you get the Validation Service up and running quickly.

## Prerequisites

- Python 3.11 or later
- pip (Python package manager)
- Git

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/validation-service.git
cd validation-service
```

### 2. Create Virtual Environment

```bash
# Create virtual environment
python -m venv .venv

# Activate (Linux/macOS)
source .venv/bin/activate

# Activate (Windows)
.venv\Scripts\activate
```

### 3. Install Dependencies

```bash
# Production dependencies
pip install -r requirements.txt

# Development dependencies (includes testing tools)
pip install -r requirements-dev.txt
```

### 4. Configure Environment

```bash
# Copy example configuration
cp .env.example .env

# Edit .env with your settings
```

## First Validation Run

### Using CLI

Run a complete validation on a project:

```bash
validate run --path /path/to/your/project
```

Run specific validation types:

```bash
# Code quality only
validate quality --path /path/to/your/project

# Tests only
validate tests --path /path/to/your/project

# Deployment checks only
validate deployment --path /path/to/your/project
```

### Using API

Start the API server:

```bash
uvicorn src.api.main:app --reload
```

Make a validation request:

```bash
curl -X POST http://localhost:8080/validate \
  -H "Content-Type: application/json" \
  -d '{"path": "/path/to/project"}'
```

## Understanding Results

### Validation Status

- `pending` - Validation queued
- `running` - Validation in progress
- `passed` - All checks passed
- `failed` - One or more checks failed
- `error` - Validation encountered an error

### Report Structure

```json
{
  "run_id": "uuid",
  "status": "passed",
  "quality": {
    "linting": {"passed": true, "violations": []},
    "complexity": {"passed": true, "issues": []},
    "coverage": {"percentage": 85.5, "passed": true}
  },
  "testing": {
    "total": 150,
    "passed": 148,
    "failed": 2,
    "suites": [...]
  },
  "deployment": {
    "config": {"passed": true},
    "dependencies": {"passed": true, "vulnerabilities": []}
  }
}
```

## Next Steps

- [Configuration Options](configuration.md) - Customize validation settings
- [CLI Reference](cli-reference.md) - Full command documentation
- [API Reference](api-reference.md) - REST API endpoints
