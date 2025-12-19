# Deployment Guide

This guide covers deploying the Validation Service in various environments.

## Table of Contents

- [Quick Start](#quick-start)
- [Docker Deployment](#docker-deployment)
- [Manual Deployment](#manual-deployment)
- [CI/CD Integration](#cicd-integration)
- [Production Considerations](#production-considerations)
- [Monitoring & Logging](#monitoring--logging)
- [Troubleshooting](#troubleshooting)

## Quick Start

### Local Development

```bash
# Install dependencies
./scripts/install.sh --dev

# Start the service
./scripts/run.sh api --reload
```

### Docker Quick Start

```bash
# Build and run
docker-compose up -d validation-service

# View logs
docker-compose logs -f validation-service
```

## Docker Deployment

### Building the Image

```bash
# Production build
docker build -t validation-service:latest .

# Development build
docker build --target development -t validation-service:dev .
```

### Running with Docker Compose

```bash
# Production mode
docker-compose up -d validation-service

# Development mode with hot reload
docker-compose --profile development up -d validation-service-dev

# Full stack with Redis and PostgreSQL
docker-compose --profile full up -d

# Run tests in container
docker-compose --profile testing run --rm test-runner
```

### Environment Variables

Configure the service using environment variables:

```bash
# Create .env file
cp .env.example .env

# Edit configuration
nano .env
```

Key environment variables:

| Variable | Description | Default |
|----------|-------------|---------|
| `VALIDATION_ENV` | Environment (development/staging/production) | development |
| `LOG_LEVEL` | Logging level | INFO |
| `PORT` | API port | 8000 |
| `ENABLE_QUALITY_VALIDATION` | Enable quality checks | true |
| `ENABLE_TEST_EXECUTION` | Enable test execution | true |
| `MIN_COVERAGE_PERCENTAGE` | Minimum code coverage | 80 |

## Manual Deployment

### Prerequisites

- Python 3.11+
- pip
- virtualenv (recommended)

### Installation Steps

```bash
# Clone repository
git clone <repository-url>
cd validation-service

# Run installation script
./scripts/install.sh

# Or manual installation
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -e .
```

### Running the Service

```bash
# Development
uvicorn src.api.main:app --reload --host 0.0.0.0 --port 8000

# Production with Gunicorn
gunicorn src.api.main:app \
    --workers 4 \
    --worker-class uvicorn.workers.UvicornWorker \
    --bind 0.0.0.0:8000
```

### Systemd Service (Linux)

Create `/etc/systemd/system/validation-service.service`:

```ini
[Unit]
Description=Validation Service
After=network.target

[Service]
Type=simple
User=www-data
Group=www-data
WorkingDirectory=/opt/validation-service
Environment="PATH=/opt/validation-service/.venv/bin"
Environment="VALIDATION_ENV=production"
ExecStart=/opt/validation-service/.venv/bin/gunicorn src.api.main:app \
    --workers 4 \
    --worker-class uvicorn.workers.UvicornWorker \
    --bind 0.0.0.0:8000
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
```

Enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable validation-service
sudo systemctl start validation-service
```

## CI/CD Integration

### GitHub Actions

The service includes a complete GitHub Actions workflow at `.github/workflows/validation.yml`:

- **Lint & Type Check**: Runs on every push
- **Unit Tests**: Runs after linting
- **Integration Tests**: Runs after unit tests
- **E2E Tests**: Runs on main branch or manual trigger
- **Security Scan**: Parallel to testing
- **Build & Package**: Creates distributable package
- **Docker Build**: Builds container image
- **Deploy**: Deploys to staging/production

### Manual CI/CD Setup

```yaml
# Example Jenkins pipeline
pipeline {
    agent any

    stages {
        stage('Test') {
            steps {
                sh 'pip install -r requirements.txt'
                sh 'pytest --cov=src'
            }
        }

        stage('Build') {
            steps {
                sh 'docker build -t validation-service .'
            }
        }

        stage('Deploy') {
            steps {
                sh 'docker-compose up -d'
            }
        }
    }
}
```

## Production Considerations

### Security

1. **Use HTTPS**: Deploy behind a reverse proxy with TLS
2. **Environment Variables**: Never commit secrets to version control
3. **Network Security**: Use firewall rules to restrict access
4. **User Permissions**: Run as non-root user

### Reverse Proxy (Nginx)

```nginx
upstream validation_service {
    server 127.0.0.1:8000;
}

server {
    listen 443 ssl http2;
    server_name validation.example.com;

    ssl_certificate /etc/letsencrypt/live/validation.example.com/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/validation.example.com/privkey.pem;

    location / {
        proxy_pass http://validation_service;
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
    }

    location /api/v1/health {
        proxy_pass http://validation_service;
        access_log off;
    }
}
```

### Scaling

For high availability:

```yaml
# docker-compose.prod.yml
services:
  validation-service:
    deploy:
      replicas: 3
      resources:
        limits:
          cpus: '1'
          memory: 1G
        reservations:
          cpus: '0.5'
          memory: 512M
```

### Database (Optional)

For persistent storage, configure PostgreSQL:

```bash
# .env
DATABASE_URL=postgresql://user:password@postgres:5432/validation_service
```

## Monitoring & Logging

### Health Checks

The service exposes health endpoints:

```bash
# Basic health check
curl http://localhost:8000/api/v1/health

# Detailed health check
curl http://localhost:8000/api/v1/health?details=true

# Readiness check
curl http://localhost:8000/api/v1/health/ready

# Liveness check
curl http://localhost:8000/api/v1/health/live
```

### Logging

Logs are structured JSON in production:

```json
{
    "timestamp": "2024-01-01T10:00:00Z",
    "level": "INFO",
    "message": "Validation completed",
    "run_id": "abc123",
    "status": "passed",
    "duration": 45.5
}
```

Configure log aggregation:

```bash
# Send to stdout for container logging
LOG_FORMAT=json

# Or configure log file
LOG_FILE=/var/log/validation-service/app.log
```

### Metrics

Export metrics for Prometheus:

```bash
curl http://localhost:8000/api/v1/health/metrics
```

## Troubleshooting

### Common Issues

**Service won't start:**
```bash
# Check logs
docker-compose logs validation-service

# Check port availability
netstat -tlnp | grep 8000
```

**Tests failing:**
```bash
# Run with verbose output
pytest -v --tb=long

# Check test environment
echo $VALIDATION_ENV
```

**Permission errors:**
```bash
# Fix file permissions
chown -R appuser:appuser /app/data
chmod 755 /app/data
```

### Debug Mode

Enable debug logging:

```bash
LOG_LEVEL=DEBUG ./scripts/run.sh api
```

### Support

For issues:
1. Check the logs
2. Review configuration
3. Consult this documentation
4. Open a GitHub issue
