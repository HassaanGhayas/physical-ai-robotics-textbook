"""Tests for flaky tests API endpoints."""

import pytest
from unittest.mock import MagicMock, patch
from uuid import uuid4

from fastapi.testclient import TestClient

from src.api.main import create_app
from src.core.models import FlakyTestRecord, FlakyStatus

# Alias for test compatibility
FlakyTestInfo = FlakyTestRecord


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
    with patch("src.api.endpoints.flaky_tests.get_storage") as mock:
        storage = MagicMock()
        mock.return_value = storage
        yield storage


@pytest.fixture
def sample_flaky_tests():
    """Create sample flaky tests."""
    return [
        FlakyTestInfo(
            test_id="test_module::test_function_1",
            pass_rate=0.75,
            sample_size=20,
            confidence=0.92,
            status=FlakyStatus.ACTIVE_FLAKY,
        ),
        FlakyTestInfo(
            test_id="test_module::test_function_2",
            pass_rate=0.85,
            sample_size=15,
            confidence=0.88,
            status=FlakyStatus.ACTIVE_FLAKY,
        ),
        FlakyTestInfo(
            test_id="test_module::test_function_3",
            pass_rate=0.95,
            sample_size=50,
            confidence=0.95,
            status=FlakyStatus.RESOLVED,
        ),
    ]


class TestFlakyTestsEndpoints:
    """Tests for flaky tests endpoints."""

    def test_list_flaky_tests(self, client, mock_storage, sample_flaky_tests):
        """Test listing flaky tests."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.get("/api/v1/flaky-tests")

        assert response.status_code == 200
        data = response.json()
        assert len(data["tests"]) == 3

    def test_list_flaky_tests_with_status_filter(self, client, mock_storage, sample_flaky_tests):
        """Test filtering flaky tests by status."""
        active = [t for t in sample_flaky_tests if t.status == FlakyStatus.ACTIVE_FLAKY]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.get("/api/v1/flaky-tests?status=active_flaky")

        assert response.status_code == 200

    def test_list_flaky_tests_with_limit(self, client, mock_storage, sample_flaky_tests):
        """Test listing flaky tests with limit."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.get("/api/v1/flaky-tests?limit=2")

        assert response.status_code == 200

    def test_list_flaky_tests_empty(self, client, mock_storage):
        """Test listing when no flaky tests exist."""
        mock_storage.get_active_flaky_tests.return_value = []

        response = client.get("/api/v1/flaky-tests")

        assert response.status_code == 200
        data = response.json()
        assert len(data["tests"]) == 0


class TestFlakyTestDetails:
    """Tests for flaky test details."""

    def test_get_flaky_test_detail(self, client, mock_storage, sample_flaky_tests):
        """Test getting flaky test details."""
        test = sample_flaky_tests[0]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.get(f"/api/v1/flaky-tests/{test.test_id}")

        assert response.status_code == 200
        data = response.json()
        assert data["test_id"] == test.test_id
        assert data["pass_rate"] == test.pass_rate

    def test_get_flaky_test_not_found(self, client, mock_storage):
        """Test getting non-existent flaky test."""
        mock_storage.get_active_flaky_tests.return_value = []

        response = client.get("/api/v1/flaky-tests/nonexistent::test")

        assert response.status_code == 404


class TestFlakyTestManagement:
    """Tests for flaky test management."""

    def test_update_flaky_test_status(self, client, mock_storage, sample_flaky_tests):
        """Test updating flaky test status."""
        test = sample_flaky_tests[0]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.patch(
            f"/api/v1/flaky-tests/{test.test_id}",
            json={"status": "resolved"},
        )

        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "resolved"

    def test_update_flaky_test_invalid_status(self, client, mock_storage, sample_flaky_tests):
        """Test updating with invalid status."""
        test = sample_flaky_tests[0]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.patch(
            f"/api/v1/flaky-tests/{test.test_id}",
            json={"status": "invalid_status"},
        )

        assert response.status_code == 400

    def test_update_flaky_test_not_found(self, client, mock_storage):
        """Test updating non-existent flaky test."""
        mock_storage.get_active_flaky_tests.return_value = []

        response = client.patch(
            "/api/v1/flaky-tests/nonexistent::test",
            json={"status": "resolved"},
        )

        assert response.status_code == 404

    def test_acknowledge_flaky_test(self, client, mock_storage, sample_flaky_tests):
        """Test acknowledging a flaky test."""
        test = sample_flaky_tests[0]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.post(f"/api/v1/flaky-tests/{test.test_id}/acknowledge")

        assert response.status_code == 200

    def test_quarantine_flaky_test(self, client, mock_storage, sample_flaky_tests):
        """Test quarantining a flaky test."""
        test = sample_flaky_tests[0]
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.post(f"/api/v1/flaky-tests/{test.test_id}/quarantine")

        assert response.status_code == 200


class TestFlakyTestStatistics:
    """Tests for flaky test statistics."""

    def test_get_flaky_stats(self, client, mock_storage, sample_flaky_tests):
        """Test getting flaky test statistics."""
        mock_storage.get_active_flaky_tests.return_value = sample_flaky_tests

        response = client.get("/api/v1/flaky-tests/stats")

        assert response.status_code == 200
        data = response.json()
        assert "total" in data
        assert "active" in data
        assert "resolved" in data
        assert "quarantined" in data

    def test_get_flaky_stats_empty(self, client, mock_storage):
        """Test statistics with no flaky tests."""
        mock_storage.get_active_flaky_tests.return_value = []

        response = client.get("/api/v1/flaky-tests/stats")

        assert response.status_code == 200
        data = response.json()
        assert data["total"] == 0
