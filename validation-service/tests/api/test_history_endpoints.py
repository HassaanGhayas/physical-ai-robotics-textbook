"""Tests for history API endpoints."""

import pytest
from datetime import datetime
from unittest.mock import MagicMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient

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
def mock_storage():
    """Mock storage for testing."""
    with patch("src.api.endpoints.history.get_storage") as mock:
        storage = MagicMock()
        mock.return_value = storage
        yield storage


@pytest.fixture
def sample_runs():
    """Create sample validation runs."""
    return [
        ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.CI_CD,
            repository="repo-1",
            branch="main",
            commit_sha="abc123",
            started_at=datetime(2024, 1, 1, 10, 0, 0),
            completed_at=datetime(2024, 1, 1, 10, 5, 0),
        ),
        ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.FAILED,
            trigger_source=TriggerSource.MANUAL,
            repository="repo-1",
            branch="feature",
            commit_sha="def456",
            started_at=datetime(2024, 1, 2, 10, 0, 0),
            completed_at=datetime(2024, 1, 2, 10, 3, 0),
        ),
        ValidationRun(
            run_id=uuid4(),
            status=ValidationStatus.PASSED,
            trigger_source=TriggerSource.MANUAL,
            repository="repo-2",
            branch="main",
            commit_sha="ghi789",
            started_at=datetime(2024, 1, 3, 10, 0, 0),
            completed_at=datetime(2024, 1, 3, 10, 10, 0),
        ),
    ]


class TestHistoryEndpoints:
    """Tests for history endpoints."""

    def test_list_runs_success(self, client, mock_storage, sample_runs):
        """Test listing validation runs."""
        mock_storage.list_runs.return_value = sample_runs

        response = client.get("/api/v1/history")

        assert response.status_code == 200
        data = response.json()
        assert len(data["runs"]) == 3

    def test_list_runs_with_limit(self, client, mock_storage, sample_runs):
        """Test listing runs with limit."""
        mock_storage.list_runs.return_value = sample_runs[:2]

        response = client.get("/api/v1/history?limit=2")

        assert response.status_code == 200
        data = response.json()
        assert len(data["runs"]) == 2

    def test_list_runs_with_repository_filter(self, client, mock_storage, sample_runs):
        """Test filtering by repository."""
        filtered = [r for r in sample_runs if r.repository == "repo-1"]
        mock_storage.list_runs.return_value = filtered

        response = client.get("/api/v1/history?repository=repo-1")

        assert response.status_code == 200
        mock_storage.list_runs.assert_called()

    def test_list_runs_with_branch_filter(self, client, mock_storage, sample_runs):
        """Test filtering by branch."""
        filtered = [r for r in sample_runs if r.branch == "main"]
        mock_storage.list_runs.return_value = filtered

        response = client.get("/api/v1/history?branch=main")

        assert response.status_code == 200

    def test_list_runs_with_status_filter(self, client, mock_storage, sample_runs):
        """Test filtering by status."""
        filtered = [r for r in sample_runs if r.status == ValidationStatus.PASSED]
        mock_storage.list_runs.return_value = filtered

        response = client.get("/api/v1/history?status=passed")

        assert response.status_code == 200

    def test_list_runs_empty(self, client, mock_storage):
        """Test listing when no runs exist."""
        mock_storage.list_runs.return_value = []

        response = client.get("/api/v1/history")

        assert response.status_code == 200
        data = response.json()
        assert len(data["runs"]) == 0


class TestHistoryComparison:
    """Tests for history comparison."""

    def test_compare_runs(self, client, mock_storage, sample_runs):
        """Test comparing two runs."""
        run1 = sample_runs[0]
        run2 = sample_runs[1]
        mock_storage.get_run.side_effect = [run1, run2]

        response = client.get(
            f"/api/v1/history/compare?run_id_1={run1.run_id}&run_id_2={run2.run_id}"
        )

        assert response.status_code == 200
        data = response.json()
        assert "run_1" in data
        assert "run_2" in data
        assert "comparison" in data

    def test_compare_runs_not_found(self, client, mock_storage):
        """Test comparison with non-existent run."""
        run_id_1 = uuid4()
        run_id_2 = uuid4()
        mock_storage.get_run.return_value = None

        response = client.get(
            f"/api/v1/history/compare?run_id_1={run_id_1}&run_id_2={run_id_2}"
        )

        assert response.status_code == 404


class TestHistoryStatistics:
    """Tests for history statistics."""

    def test_get_statistics(self, client, mock_storage, sample_runs):
        """Test getting statistics."""
        mock_storage.list_runs.return_value = sample_runs

        response = client.get("/api/v1/history/stats")

        assert response.status_code == 200
        data = response.json()
        assert "total_runs" in data
        assert "pass_rate" in data
        assert "average_duration" in data

    def test_get_statistics_with_repository(self, client, mock_storage, sample_runs):
        """Test statistics for specific repository."""
        filtered = [r for r in sample_runs if r.repository == "repo-1"]
        mock_storage.list_runs.return_value = filtered

        response = client.get("/api/v1/history/stats?repository=repo-1")

        assert response.status_code == 200

    def test_get_statistics_empty(self, client, mock_storage):
        """Test statistics with no runs."""
        mock_storage.list_runs.return_value = []

        response = client.get("/api/v1/history/stats")

        assert response.status_code == 200
        data = response.json()
        assert data["total_runs"] == 0
