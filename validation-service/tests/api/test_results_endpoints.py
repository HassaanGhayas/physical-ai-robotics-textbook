"""Tests for results API endpoints."""

import pytest
from datetime import datetime
from unittest.mock import MagicMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient

from src.api.main import create_app
from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
    CodeQualityReport as QualityReport,
    TestSuiteResult as TestResult,
    TestSuiteType as SuiteType,
    DeploymentChecklist,
)


@pytest.fixture
def app():
    """Create test application."""
    return create_app()


@pytest.fixture
def client(app):
    """Create test client."""
    return TestClient(app)


@pytest.fixture
def mock_storage():
    """Mock storage for testing."""
    with patch("src.api.endpoints.results.get_storage") as mock:
        storage = MagicMock()
        mock.return_value = storage
        yield storage


@pytest.fixture
def sample_run():
    """Create sample validation run with full results."""
    run_id = uuid4()
    return ValidationRun(
        run_id=run_id,
        status=ValidationStatus.PASSED,
        trigger_source=TriggerSource.MANUAL,
        repository="test-repo",
        branch="main",
        commit_sha="abc123def456",
        started_at=datetime(2024, 1, 1, 10, 0, 0),
        completed_at=datetime(2024, 1, 1, 10, 5, 0),
        duration_seconds=300.0,
        quality_report=QualityReport(
            coverage_percentage=85.0,
            passed=True,
            issues=[],
        ),
        test_results=[
            TestResult(
                suite_type=SuiteType.UNIT,
                total_tests=100,
                passed_tests=98,
                failed_tests=2,
                skipped_tests=0,
                duration_seconds=30.0,
                passed=True,
                failures=[],
            ),
            TestResult(
                suite_type=SuiteType.INTEGRATION,
                total_tests=50,
                passed_tests=50,
                failed_tests=0,
                skipped_tests=0,
                duration_seconds=60.0,
                passed=True,
                failures=[],
            ),
        ],
        deployment_checklist=DeploymentChecklist(
            passed=True,
            checks=[],
        ),
    )


class TestResultsEndpoints:
    """Tests for results endpoints."""

    def test_get_results_success(self, client, mock_storage, sample_run):
        """Test successful results retrieval."""
        mock_storage.get_run.return_value = sample_run

        response = client.get(f"/api/v1/results/{sample_run.run_id}")

        assert response.status_code == 200
        data = response.json()
        assert data["run_id"] == str(sample_run.run_id)
        assert data["status"] == "passed"
        assert data["repository"] == "test-repo"
        assert data["branch"] == "main"
        assert data["quality_report"] is not None
        assert len(data["test_results"]) == 2

    def test_get_results_not_found(self, client, mock_storage):
        """Test results for non-existent run."""
        mock_storage.get_run.return_value = None
        run_id = uuid4()

        response = client.get(f"/api/v1/results/{run_id}")

        assert response.status_code == 404
        assert "not found" in response.json()["detail"]

    def test_get_results_invalid_uuid(self, client):
        """Test results with invalid UUID."""
        response = client.get("/api/v1/results/invalid-uuid")

        assert response.status_code == 422

    def test_get_status_success(self, client, mock_storage, sample_run):
        """Test status retrieval."""
        mock_storage.get_run.return_value = sample_run

        response = client.get(f"/api/v1/results/{sample_run.run_id}/status")

        assert response.status_code == 200
        data = response.json()
        assert data["run_id"] == str(sample_run.run_id)
        assert data["status"] == "passed"
        assert "progress" in data
        assert "quality" in data["progress"]["completed_stages"]
        assert "unit_tests" in data["progress"]["completed_stages"]

    def test_get_status_not_found(self, client, mock_storage):
        """Test status for non-existent run."""
        mock_storage.get_run.return_value = None
        run_id = uuid4()

        response = client.get(f"/api/v1/results/{run_id}/status")

        assert response.status_code == 404


class TestResultsWithPartialData:
    """Tests for results with partial data."""

    def test_results_without_quality_report(self, client, mock_storage):
        """Test results without quality report."""
        run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        mock_storage.get_run.return_value = run

        response = client.get(f"/api/v1/results/{run.run_id}")

        assert response.status_code == 200
        data = response.json()
        assert data["quality_report"] is None
        assert data["test_results"] == []
        assert data["deployment_checklist"] is None

    def test_results_without_timestamps(self, client, mock_storage):
        """Test results without timestamps."""
        run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.RUNNING,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        mock_storage.get_run.return_value = run

        response = client.get(f"/api/v1/results/{run.run_id}")

        assert response.status_code == 200
        data = response.json()
        assert data["started_at"] is None
        assert data["completed_at"] is None
        assert data["duration_seconds"] is None


class TestStatusProgress:
    """Tests for status progress tracking."""

    def test_status_with_all_stages_complete(self, client, mock_storage, sample_run):
        """Test status with all stages complete."""
        mock_storage.get_run.return_value = sample_run

        response = client.get(f"/api/v1/results/{sample_run.run_id}/status")

        data = response.json()
        stages = data["progress"]["completed_stages"]
        assert "quality" in stages
        assert "unit_tests" in stages
        assert "integration_tests" in stages
        assert "deployment" in stages

    def test_status_with_e2e_tests(self, client, mock_storage):
        """Test status with E2E tests."""
        run = ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
            test_results=[
                TestResult(
                    suite_type=SuiteType.E2E,
                    total_tests=10,
                    passed_tests=10,
                    failed_tests=0,
                    skipped_tests=0,
                    duration_seconds=120.0,
                    passed=True,
                    failures=[],
                ),
            ],
        )
        mock_storage.get_run.return_value = run

        response = client.get(f"/api/v1/results/{run.run_id}/status")

        data = response.json()
        assert "e2e_tests" in data["progress"]["completed_stages"]
