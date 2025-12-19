"""End-to-end tests for API workflow."""

import pytest
from unittest.mock import MagicMock, AsyncMock, patch
from uuid import uuid4
from datetime import datetime

from fastapi.testclient import TestClient

from src.api.main import create_app
from src.core.models import (
    ValidationStatus,
    TriggerSource,
    ValidationRun,
    QualityReport,
    TestResult,
    SuiteType,
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
def mock_pipeline():
    """Mock pipeline for testing."""
    with patch("src.api.endpoints.validation.get_pipeline") as mock:
        pipeline = MagicMock()
        mock.return_value = pipeline
        yield pipeline


@pytest.fixture
def mock_storage():
    """Mock storage for testing."""
    with patch("src.api.endpoints.results.get_storage") as results_mock, \
         patch("src.api.endpoints.history.get_storage") as history_mock, \
         patch("src.api.endpoints.flaky_tests.get_storage") as flaky_mock:

        storage = MagicMock()
        results_mock.return_value = storage
        history_mock.return_value = storage
        flaky_mock.return_value = storage
        yield storage


class TestValidationWorkflow:
    """End-to-end tests for validation workflow."""

    def test_full_validation_workflow(self, client, mock_pipeline, mock_storage):
        """Test complete validation workflow: submit -> check status -> get results."""
        run_id = uuid4()

        # Create validation run that goes through stages
        pending_run = ValidationRun(
            run_id=run_id,
            status=ValidationStatus.RUNNING,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
            started_at=datetime.utcnow(),
        )

        completed_run = ValidationRun(
            run_id=run_id,
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
            started_at=datetime.utcnow(),
            completed_at=datetime.utcnow(),
            duration_seconds=60.0,
            quality_report=QualityReport(
                coverage_percentage=85.0,
                passed=True,
                issues=[],
            ),
            test_results=[
                TestResult(
                    suite_type=SuiteType.UNIT,
                    total_tests=100,
                    passed_tests=100,
                    failed_tests=0,
                    skipped_tests=0,
                    duration_seconds=30.0,
                    passed=True,
                    failures=[],
                ),
            ],
        )

        # Step 1: Submit validation
        mock_pipeline.run = AsyncMock(return_value=completed_run)

        submit_response = client.post(
            "/api/v1/validate",
            json={
                "project_path": "/tmp/test",
                "repository": "test-repo",
                "branch": "main",
                "commit_sha": "abc123",
            },
        )

        assert submit_response.status_code == 200
        submit_data = submit_response.json()
        assert submit_data["run_id"] == str(run_id)

        # Step 2: Check status
        mock_storage.get_run.return_value = completed_run

        status_response = client.get(f"/api/v1/results/{run_id}/status")

        assert status_response.status_code == 200
        status_data = status_response.json()
        assert status_data["status"] == "passed"

        # Step 3: Get full results
        results_response = client.get(f"/api/v1/results/{run_id}")

        assert results_response.status_code == 200
        results_data = results_response.json()
        assert results_data["status"] == "passed"
        assert results_data["quality_report"] is not None
        assert len(results_data["test_results"]) == 1

    def test_validation_history_workflow(self, client, mock_storage):
        """Test validation history workflow: list runs -> get stats."""
        runs = [
            ValidationRun(
                run_id=uuid4(),
                status=ValidationStatus.PASSED,
                trigger_source=TriggerSource.CI_CD,
                repository="repo-1",
                branch="main",
                commit_sha=f"commit{i}",
                duration_seconds=float(60 + i * 10),
            )
            for i in range(5)
        ]

        mock_storage.list_runs.return_value = runs

        # Step 1: List history
        history_response = client.get("/api/v1/history")

        assert history_response.status_code == 200
        history_data = history_response.json()
        assert len(history_data["runs"]) == 5

        # Step 2: Get statistics
        stats_response = client.get("/api/v1/history/stats")

        assert stats_response.status_code == 200
        stats_data = stats_response.json()
        assert stats_data["total_runs"] == 5
        assert stats_data["pass_rate"] == 100.0


class TestQualityOnlyWorkflow:
    """Tests for quality-only validation workflow."""

    def test_quality_only_workflow(self, client, mock_pipeline, mock_storage):
        """Test quality-only validation workflow."""
        run_id = uuid4()

        run = ValidationRun(
            run_id=run_id,
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
            quality_report=QualityReport(
                coverage_percentage=90.0,
                passed=True,
                issues=[],
            ),
        )

        mock_pipeline.run_quality_only = AsyncMock(return_value=run)

        # Submit quality-only validation
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
        assert data["status"] == "passed"


class TestFlakyTestWorkflow:
    """Tests for flaky test management workflow."""

    def test_flaky_test_management_workflow(self, client, mock_storage):
        """Test flaky test management workflow: list -> update -> verify."""
        from src.core.models import FlakyTestInfo, FlakyStatus

        flaky_tests = [
            FlakyTestInfo(
                test_id="test_module::test_flaky",
                pass_rate=0.75,
                sample_size=20,
                confidence=0.92,
                status=FlakyStatus.ACTIVE_FLAKY,
            ),
        ]

        mock_storage.get_active_flaky_tests.return_value = flaky_tests

        # Step 1: List flaky tests
        list_response = client.get("/api/v1/flaky-tests")

        assert list_response.status_code == 200
        list_data = list_response.json()
        assert len(list_data["tests"]) == 1

        # Step 2: Get statistics
        stats_response = client.get("/api/v1/flaky-tests/stats")

        assert stats_response.status_code == 200
        stats_data = stats_response.json()
        assert stats_data["active"] == 1

        # Step 3: Update status
        update_response = client.patch(
            "/api/v1/flaky-tests/test_module::test_flaky",
            json={"status": "resolved"},
        )

        assert update_response.status_code == 200


class TestHealthWorkflow:
    """Tests for health check workflow."""

    def test_health_check_workflow(self, client):
        """Test health check workflow."""
        # Step 1: Basic health check
        health_response = client.get("/api/v1/health")

        assert health_response.status_code == 200
        assert health_response.json()["status"] == "healthy"

        # Step 2: Readiness check
        ready_response = client.get("/api/v1/health/ready")

        assert ready_response.status_code == 200
        assert ready_response.json()["ready"] is True

        # Step 3: Liveness check
        live_response = client.get("/api/v1/health/live")

        assert live_response.status_code == 200
        assert live_response.json()["alive"] is True


class TestErrorHandling:
    """Tests for error handling in workflows."""

    def test_validation_error_workflow(self, client, mock_pipeline):
        """Test error handling during validation."""
        mock_pipeline.run = AsyncMock(side_effect=Exception("Validation failed"))

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

    def test_not_found_workflow(self, client, mock_storage):
        """Test not found error handling."""
        mock_storage.get_run.return_value = None
        run_id = uuid4()

        response = client.get(f"/api/v1/results/{run_id}")

        assert response.status_code == 404
        assert "not found" in response.json()["detail"]
