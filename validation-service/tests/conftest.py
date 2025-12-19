"""Pytest configuration and fixtures."""

import os
import sys
import pytest
import asyncio
from pathlib import Path
from unittest.mock import MagicMock, AsyncMock, patch
from uuid import uuid4
from datetime import datetime
from typing import Generator, Any

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


# ============================================================================
# Async Configuration
# ============================================================================

@pytest.fixture(scope="session")
def event_loop():
    """Create an instance of the default event loop for the test session."""
    loop = asyncio.get_event_loop_policy().new_event_loop()
    yield loop
    loop.close()


# ============================================================================
# Environment Fixtures
# ============================================================================

@pytest.fixture(autouse=True)
def reset_environment():
    """Reset environment variables before each test."""
    original_env = os.environ.copy()
    yield
    os.environ.clear()
    os.environ.update(original_env)


@pytest.fixture
def test_env():
    """Set up test environment variables."""
    os.environ["VALIDATION_ENV"] = "test"
    os.environ["LOG_LEVEL"] = "DEBUG"
    return os.environ


# ============================================================================
# Storage Fixtures
# ============================================================================

@pytest.fixture
def mock_storage():
    """Create a mock storage instance."""
    storage = MagicMock()
    storage.get_run = MagicMock(return_value=None)
    storage.save_run = MagicMock()
    storage.list_runs = MagicMock(return_value=[])
    storage.get_latest_run = MagicMock(return_value=None)
    storage.get_active_flaky_tests = MagicMock(return_value=[])
    storage.save = MagicMock()
    return storage


@pytest.fixture
def storage_patch(mock_storage):
    """Patch storage in all common locations."""
    with patch("src.core.storage.get_storage", return_value=mock_storage), \
         patch("src.api.endpoints.results.get_storage", return_value=mock_storage), \
         patch("src.api.endpoints.history.get_storage", return_value=mock_storage), \
         patch("src.api.endpoints.flaky_tests.get_storage", return_value=mock_storage), \
         patch("src.cli.commands.results.get_storage", return_value=mock_storage), \
         patch("src.cli.commands.status.get_storage", return_value=mock_storage), \
         patch("src.cli.commands.flaky_tests.get_storage", return_value=mock_storage):
        yield mock_storage


# ============================================================================
# Model Fixtures
# ============================================================================

@pytest.fixture
def run_id():
    """Generate a random run ID."""
    return uuid4()


@pytest.fixture
def sample_validation_run(run_id):
    """Create a sample validation run."""
    from src.core.models import ValidationStatus, TriggerSource, ValidationRun

    return ValidationRun(
        run_id=run_id,
        status=ValidationStatus.PASSED,
        trigger_source=TriggerSource.MANUAL,
        repository="test-repo",
        branch="main",
        commit_sha="abc123def456789",
        started_at=datetime.utcnow(),
        completed_at=datetime.utcnow(),
        duration_seconds=45.5,
    )


@pytest.fixture
def sample_quality_report():
    """Create a sample quality report."""
    from src.core.models import QualityReport, QualityIssue, IssueSeverity

    return QualityReport(
        coverage_percentage=85.0,
        passed=True,
        issues=[
            QualityIssue(
                severity=IssueSeverity.WARNING,
                message="Function 'process_data' has high complexity",
                file_path="src/processing.py",
                line_number=42,
            ),
        ],
    )


@pytest.fixture
def sample_test_result():
    """Create a sample test result."""
    from src.core.models import TestResult, SuiteType, TestFailure

    return TestResult(
        suite_type=SuiteType.UNIT,
        total_tests=100,
        passed_tests=98,
        failed_tests=2,
        skipped_tests=0,
        duration_seconds=30.5,
        passed=False,
        failures=[
            TestFailure(
                test_name="test_edge_case",
                error_message="AssertionError: Expected True, got False",
                file_path="tests/test_processing.py",
                line_number=55,
            ),
        ],
    )


@pytest.fixture
def sample_flaky_test():
    """Create a sample flaky test info."""
    from src.core.models import FlakyTestInfo, FlakyStatus

    return FlakyTestInfo(
        test_id="test_module::TestClass::test_method",
        pass_rate=0.75,
        sample_size=20,
        confidence=0.92,
        status=FlakyStatus.ACTIVE_FLAKY,
    )


# ============================================================================
# Project Fixtures
# ============================================================================

@pytest.fixture
def temp_project(tmp_path):
    """Create a temporary project structure."""
    # Create directories
    src_dir = tmp_path / "src"
    src_dir.mkdir()
    tests_dir = tmp_path / "tests"
    tests_dir.mkdir()
    git_dir = tmp_path / ".git"
    git_dir.mkdir()

    # Create files
    (src_dir / "__init__.py").write_text("")
    (src_dir / "main.py").write_text('def main():\n    return "Hello, World!"')
    (tests_dir / "__init__.py").write_text("")
    (tests_dir / "test_main.py").write_text(
        'from src.main import main\n\ndef test_main():\n    assert main() == "Hello, World!"'
    )

    # Create pyproject.toml
    (tmp_path / "pyproject.toml").write_text(
        '[project]\nname = "test-project"\nversion = "1.0.0"'
    )

    return tmp_path


@pytest.fixture
def git_info():
    """Create sample git info."""
    return {
        "is_git_repo": True,
        "repository": "test-repo",
        "branch": "main",
        "commit_sha": "abc123def456789",
    }


# ============================================================================
# Settings Fixtures
# ============================================================================

@pytest.fixture
def mock_settings():
    """Create mock settings."""
    settings = MagicMock()
    settings.environment = "test"
    settings.log_level = "DEBUG"
    settings.enable_quality_validation = True
    settings.enable_test_execution = True
    settings.enable_deployment_validation = True
    settings.pipeline_timeout_minutes = 15

    settings.quality = MagicMock()
    settings.quality.min_coverage_percentage = 80.0
    settings.quality.max_cyclomatic_complexity = 10
    settings.quality.max_line_length = 120

    settings.testing = MagicMock()
    settings.testing.parallel_execution = True
    settings.testing.flaky_detection_enabled = True
    settings.testing.retry_count = 3

    settings.deployment = MagicMock()
    settings.deployment.require_passing_tests = True
    settings.deployment.require_passing_quality = True

    return settings


@pytest.fixture
def settings_patch(mock_settings):
    """Patch settings in all common locations."""
    with patch("src.core.config.get_settings", return_value=mock_settings), \
         patch("src.cli.commands.config.get_settings", return_value=mock_settings), \
         patch("src.cli.commands.status.get_settings", return_value=mock_settings):
        yield mock_settings


# ============================================================================
# Pipeline Fixtures
# ============================================================================

@pytest.fixture
def mock_pipeline(sample_validation_run):
    """Create a mock pipeline."""
    pipeline = MagicMock()
    pipeline.run = AsyncMock(return_value=sample_validation_run)
    pipeline.run_quality_only = AsyncMock(return_value=sample_validation_run)
    pipeline.run_tests_only = AsyncMock(return_value=sample_validation_run)
    pipeline.run_deployment_only = AsyncMock(return_value=sample_validation_run)
    return pipeline


@pytest.fixture
def pipeline_patch(mock_pipeline):
    """Patch pipeline in all common locations."""
    with patch("src.core.pipeline.get_pipeline", return_value=mock_pipeline), \
         patch("src.api.endpoints.validation.get_pipeline", return_value=mock_pipeline), \
         patch("src.cli.commands.run.get_pipeline", return_value=mock_pipeline):
        yield mock_pipeline


# ============================================================================
# API Fixtures
# ============================================================================

@pytest.fixture
def api_app():
    """Create test API application."""
    from src.api.main import create_app
    return create_app()


@pytest.fixture
def api_client(api_app):
    """Create test API client."""
    from fastapi.testclient import TestClient
    return TestClient(api_app)


# ============================================================================
# CLI Fixtures
# ============================================================================

@pytest.fixture
def cli_runner():
    """Create CLI test runner."""
    from typer.testing import CliRunner
    return CliRunner()


# ============================================================================
# Marker Helpers
# ============================================================================

def pytest_configure(config):
    """Configure custom markers."""
    config.addinivalue_line("markers", "unit: mark test as unit test")
    config.addinivalue_line("markers", "integration: mark test as integration test")
    config.addinivalue_line("markers", "e2e: mark test as end-to-end test")
    config.addinivalue_line("markers", "slow: mark test as slow")
    config.addinivalue_line("markers", "flaky: mark test as known flaky")
