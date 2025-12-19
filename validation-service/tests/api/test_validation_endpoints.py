"""Tests for validation API endpoints."""

import pytest
from unittest.mock import AsyncMock, MagicMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient
from httpx import ASGITransport, AsyncClient

from src.api.main import create_app
from src.core.models import ValidationStatus, TriggerSource, ValidationRun


@pytest.fixture
def app():
    """Create test application."""
    return create_app()


@pytest.fixture
def client(app):
    """Create test client."""
    return TestClient(app)


@pytest.fixture
def mock_pipeline():
    """Mock pipeline for testing."""
    with patch("src.api.endpoints.validation.get_pipeline") as mock:
        pipeline = MagicMock()
        mock.return_value = pipeline
        yield pipeline


@pytest.fixture
def sample_run():
    """Create sample validation run."""
    return ValidationRun(
        run_id=uuid4(),
        status=ValidationStatus.PASSED,
        trigger_source=TriggerSource.MANUAL,
        repository="test-repo",
        branch="main",
        commit_sha="abc123",
    )


class TestValidationEndpoints:
    """Tests for validation endpoints."""

    def test_run_validation_success(self, client, mock_pipeline, sample_run):
        """Test successful validation run."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert data["run_id"] == str(sample_run.run_id)
        assert data["status"] == "passed"

    def test_run_validation_with_options(self, client, mock_pipeline, sample_run):
        """Test validation with custom options."""
        mock_pipeline.run = AsyncMock(return_value=sample_run)

        response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "feature",
                "commit_sha": "def456",
                "run_quality": True,
                "run_tests": False,
                "run_deployment": False,
                "fail_fast": True,
                "timeout_minutes": 10,
            },
        )

        assert response.status_code == 200
        mock_pipeline.run.assert_called_once()

    def test_run_validation_missing_fields(self, client):
        """Test validation with missing required fields."""
        response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                # Missing repository, branch, commit_sha
            },
        )

        assert response.status_code == 422

    def test_run_quality_validation(self, client, mock_pipeline, sample_run):
        """Test quality-only validation."""
        mock_pipeline.run_quality_only = AsyncMock(return_value=sample_run)

        response = client.post(
            "/api/v1/validate/quality",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "Quality validation" in data["message"]

    def test_run_test_validation(self, client, mock_pipeline, sample_run):
        """Test test-only validation."""
        mock_pipeline.run_tests_only = AsyncMock(return_value=sample_run)

        response = client.post(
            "/api/v1/validate/tests",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "Test execution" in data["message"]

    def test_run_deployment_validation(self, client, mock_pipeline, sample_run):
        """Test deployment-only validation."""
        mock_pipeline.run_deployment_only = AsyncMock(return_value=sample_run)

        response = client.post(
            "/api/v1/validate/deployment",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert "Deployment validation" in data["message"]


class TestValidationFailures:
    """Tests for validation failure scenarios."""

    def test_validation_pipeline_error(self, client, mock_pipeline):
        """Test handling of pipeline errors."""
        mock_pipeline.run = AsyncMock(side_effect=Exception("Pipeline failed"))

        response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 500

    def test_validation_returns_failed_status(self, client, mock_pipeline):
        """Test handling of failed validation."""
        failed_run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        mock_pipeline.run = AsyncMock(return_value=failed_run)

        response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "failed"
