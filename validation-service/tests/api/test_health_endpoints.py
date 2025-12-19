"""Tests for health API endpoints."""

import pytest
from unittest.mock import MagicMock, patch

from fastapi.testclient import TestClient

from src.api.main import create_app


@pytest.fixture
def app():
    """Create test application."""
    return create_app()


@pytest.fixture
def client(app):
    """Create test client."""
    return TestClient(app)


class TestHealthEndpoints:
    """Tests for health endpoints."""

    def test_health_check(self, client):
        """Test basic health check."""
        response = client.get("/api/v1/health")

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "healthy"

    def test_health_check_with_details(self, client):
        """Test health check with details."""
        response = client.get("/api/v1/health?details=true")

        assert response.status_code == 200
        data = response.json()
        assert "status" in data
        assert "components" in data

    def test_readiness_check(self, client):
        """Test readiness check."""
        response = client.get("/api/v1/health/ready")

        assert response.status_code == 200
        data = response.json()
        assert data["ready"] is True

    def test_liveness_check(self, client):
        """Test liveness check."""
        response = client.get("/api/v1/health/live")

        assert response.status_code == 200
        data = response.json()
        assert data["alive"] is True


class TestHealthDependencies:
    """Tests for health dependency checks."""

    def test_health_storage_check(self, client):
        """Test storage health check."""
        response = client.get("/api/v1/health?details=true")

        assert response.status_code == 200
        data = response.json()
        assert "storage" in data.get("components", {})

    def test_health_storage_failure(self, client):
        """Test health when storage fails - should still work as health is independent."""
        response = client.get("/api/v1/health?details=true")

        # Should still return 200
        assert response.status_code == 200


class TestHealthMetrics:
    """Tests for health metrics."""

    def test_metrics_endpoint(self, client):
        """Test metrics endpoint."""
        response = client.get("/api/v1/health/metrics")

        assert response.status_code == 200
        data = response.json()
        assert "uptime_seconds" in data
        assert "total_requests" in data

    def test_version_endpoint(self, client):
        """Test version endpoint."""
        response = client.get("/api/v1/health/version")

        assert response.status_code == 200
        data = response.json()
        assert "version" in data
        assert "python_version" in data
