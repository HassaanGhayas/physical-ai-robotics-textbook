"""End-to-end tests for full validation pipeline."""

import pytest
import asyncio
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock
from uuid import uuid4

from src.core.models import ValidationStatus, TriggerSource
from src.core.pipeline import ValidationPipeline, PipelineConfig


@pytest.fixture
def sample_project(tmp_path):
    """Create a sample project for testing."""
    # Create Python project structure
    src_dir = tmp_path / "src"
    src_dir.mkdir()

    # Main module
    main_file = src_dir / "main.py"
    main_file.write_text('''
"""Main module."""

def greet(name: str) -> str:
    """Return a greeting."""
    return f"Hello, {name}!"

def add(a: int, b: int) -> int:
    """Add two numbers."""
    return a + b
''')

    # Tests directory
    tests_dir = tmp_path / "tests"
    tests_dir.mkdir()

    test_file = tests_dir / "test_main.py"
    test_file.write_text('''
"""Tests for main module."""

from src.main import greet, add

def test_greet():
    assert greet("World") == "Hello, World!"

def test_add():
    assert add(1, 2) == 3
''')

    # pyproject.toml
    pyproject = tmp_path / "pyproject.toml"
    pyproject.write_text('''
[project]
name = "test-project"
version = "1.0.0"

[tool.pytest.ini_options]
testpaths = ["tests"]
''')

    # Git initialization
    git_dir = tmp_path / ".git"
    git_dir.mkdir()

    return tmp_path


@pytest.fixture
def mock_quality_validators():
    """Mock quality validators."""
    with patch("src.quality.validators.coverage.CoverageValidator") as cov_mock, \
         patch("src.quality.validators.linting.LintingValidator") as lint_mock, \
         patch("src.quality.validators.complexity.ComplexityValidator") as comp_mock, \
         patch("src.quality.validators.security.SecurityValidator") as sec_mock:

        cov_mock.return_value.validate = AsyncMock(return_value=MagicMock(
            passed=True,
            coverage_percentage=85.0,
            issues=[],
        ))
        lint_mock.return_value.validate = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))
        comp_mock.return_value.validate = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))
        sec_mock.return_value.validate = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))

        yield {
            "coverage": cov_mock,
            "linting": lint_mock,
            "complexity": comp_mock,
            "security": sec_mock,
        }


@pytest.fixture
def mock_test_runners():
    """Mock test runners."""
    with patch("src.testing.runners.unit_runner.UnitTestRunner") as unit_mock, \
         patch("src.testing.runners.integration_runner.IntegrationTestRunner") as int_mock, \
         patch("src.testing.runners.e2e_runner.E2ETestRunner") as e2e_mock:

        unit_mock.return_value.run = AsyncMock(return_value=MagicMock(
            passed=True,
            total_tests=10,
            passed_tests=10,
            failed_tests=0,
            skipped_tests=0,
            duration_seconds=5.0,
            failures=[],
        ))
        int_mock.return_value.run = AsyncMock(return_value=MagicMock(
            passed=True,
            total_tests=5,
            passed_tests=5,
            failed_tests=0,
            skipped_tests=0,
            duration_seconds=10.0,
            failures=[],
        ))
        e2e_mock.return_value.run = AsyncMock(return_value=MagicMock(
            passed=True,
            total_tests=3,
            passed_tests=3,
            failed_tests=0,
            skipped_tests=0,
            duration_seconds=30.0,
            failures=[],
        ))

        yield {
            "unit": unit_mock,
            "integration": int_mock,
            "e2e": e2e_mock,
        }


@pytest.fixture
def mock_deployment_validators():
    """Mock deployment validators."""
    with patch("src.deployment.validators.config_validator.ConfigValidator") as config_mock, \
         patch("src.deployment.validators.dependency_checker.DependencyChecker") as dep_mock, \
         patch("src.deployment.validators.security_scanner.SecurityScanner") as sec_mock, \
         patch("src.deployment.validators.compatibility.CompatibilityChecker") as compat_mock:

        config_mock.return_value.validate = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))
        dep_mock.return_value.check = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))
        sec_mock.return_value.scan = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))
        compat_mock.return_value.check = AsyncMock(return_value=MagicMock(
            passed=True,
            issues=[],
        ))

        yield {
            "config": config_mock,
            "dependency": dep_mock,
            "security": sec_mock,
            "compatibility": compat_mock,
        }


class TestFullPipeline:
    """End-to-end tests for full pipeline."""

    @pytest.mark.asyncio
    async def test_full_pipeline_success(
        self,
        sample_project,
        mock_quality_validators,
        mock_test_runners,
        mock_deployment_validators,
    ):
        """Test successful full pipeline execution."""
        with patch("src.core.pipeline.get_storage") as storage_mock:
            storage_mock.return_value = MagicMock()

            pipeline = ValidationPipeline()
            config = PipelineConfig(
                run_quality=True,
                run_tests=True,
                run_deployment=True,
            )

            run = await pipeline.run(
                project_path=str(sample_project),
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                trigger_source=TriggerSource.MANUAL,
                config=config,
            )

            assert run.status == ValidationStatus.PASSED

    @pytest.mark.asyncio
    async def test_pipeline_quality_only(
        self,
        sample_project,
        mock_quality_validators,
    ):
        """Test quality-only pipeline execution."""
        with patch("src.core.pipeline.get_storage") as storage_mock:
            storage_mock.return_value = MagicMock()

            pipeline = ValidationPipeline()
            config = PipelineConfig(
                run_quality=True,
                run_tests=False,
                run_deployment=False,
            )

            run = await pipeline.run(
                project_path=str(sample_project),
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                trigger_source=TriggerSource.MANUAL,
                config=config,
            )

            assert run.status == ValidationStatus.PASSED
            assert run.quality_report is not None

    @pytest.mark.asyncio
    async def test_pipeline_tests_only(
        self,
        sample_project,
        mock_test_runners,
    ):
        """Test tests-only pipeline execution."""
        with patch("src.core.pipeline.get_storage") as storage_mock:
            storage_mock.return_value = MagicMock()

            pipeline = ValidationPipeline()
            config = PipelineConfig(
                run_quality=False,
                run_tests=True,
                run_deployment=False,
            )

            run = await pipeline.run(
                project_path=str(sample_project),
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                trigger_source=TriggerSource.MANUAL,
                config=config,
            )

            assert run.status == ValidationStatus.PASSED
            assert len(run.test_results) > 0


class TestPipelineFailures:
    """Tests for pipeline failure scenarios."""

    @pytest.mark.asyncio
    async def test_pipeline_quality_failure(self, sample_project):
        """Test pipeline with quality failure."""
        with patch("src.core.pipeline.get_storage") as storage_mock, \
             patch("src.quality.validators.coverage.CoverageValidator") as cov_mock:

            storage_mock.return_value = MagicMock()
            cov_mock.return_value.validate = AsyncMock(return_value=MagicMock(
                passed=False,
                coverage_percentage=50.0,
                issues=["Coverage below threshold"],
            ))

            pipeline = ValidationPipeline()
            config = PipelineConfig(
                run_quality=True,
                run_tests=False,
                run_deployment=False,
                fail_fast=True,
            )

            run = await pipeline.run(
                project_path=str(sample_project),
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                trigger_source=TriggerSource.MANUAL,
                config=config,
            )

            assert run.status == ValidationStatus.FAILED

    @pytest.mark.asyncio
    async def test_pipeline_test_failure(self, sample_project):
        """Test pipeline with test failure."""
        with patch("src.core.pipeline.get_storage") as storage_mock, \
             patch("src.testing.runners.unit_runner.UnitTestRunner") as unit_mock:

            storage_mock.return_value = MagicMock()
            unit_mock.return_value.run = AsyncMock(return_value=MagicMock(
                passed=False,
                total_tests=10,
                passed_tests=8,
                failed_tests=2,
                skipped_tests=0,
                duration_seconds=5.0,
                failures=["test_1 failed", "test_2 failed"],
            ))

            pipeline = ValidationPipeline()
            config = PipelineConfig(
                run_quality=False,
                run_tests=True,
                run_deployment=False,
            )

            run = await pipeline.run(
                project_path=str(sample_project),
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                trigger_source=TriggerSource.MANUAL,
                config=config,
            )

            assert run.status == ValidationStatus.FAILED


class TestPipelineTimeout:
    """Tests for pipeline timeout handling."""

    @pytest.mark.asyncio
    async def test_pipeline_timeout(self, sample_project):
        """Test pipeline timeout handling."""
        with patch("src.core.pipeline.get_storage") as storage_mock:
            storage_mock.return_value = MagicMock()

            async def slow_validation(*args, **kwargs):
                await asyncio.sleep(10)  # Simulate slow validation

            with patch("src.quality.validators.coverage.CoverageValidator") as cov_mock:
                cov_mock.return_value.validate = slow_validation

                pipeline = ValidationPipeline()
                config = PipelineConfig(
                    run_quality=True,
                    run_tests=False,
                    run_deployment=False,
                    timeout_minutes=0.001,  # Very short timeout
                )

                run = await pipeline.run(
                    project_path=str(sample_project),
                    repository="test-repo",
                    branch="main",
                    commit_sha="abc123",
                    trigger_source=TriggerSource.MANUAL,
                    config=config,
                )

                # Should handle timeout gracefully
                assert run.status in [ValidationStatus.FAILED, ValidationStatus.ERROR]
