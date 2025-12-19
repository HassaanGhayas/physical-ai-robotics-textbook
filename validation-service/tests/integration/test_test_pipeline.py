"""Integration tests for test execution pipeline.

Tests the integration between test orchestration and the validation pipeline.
"""

import pytest
import asyncio
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock
from uuid import uuid4

from src.core.models import (
    TestSuiteType,
    TestStatus,
    TriggerSource,
    ValidationStatus,
    CheckStatus,
)
from src.core.pipeline import (
    ValidationPipeline,
    PipelineConfig,
    PipelineStage,
    get_pipeline,
)
from src.testing.orchestrator import TestOrchestrator
from src.testing.models import TestSuiteExecution, TestRunnerStatus
from src.testing.pipeline_integration import (
    TestPipelineIntegration,
    get_test_pipeline_integration,
    setup_test_pipeline,
)


class TestTestPipelineIntegration:
    """Integration tests for test pipeline integration."""

    @pytest.fixture
    def mock_storage(self):
        """Create mock storage."""
        storage = MagicMock()
        storage.save_run = MagicMock()
        storage.load = MagicMock(return_value=None)
        storage.list = MagicMock(return_value=[])
        return storage

    @pytest.fixture
    def mock_orchestrator(self):
        """Create mock test orchestrator."""
        orchestrator = MagicMock(spec=TestOrchestrator)

        def mock_run_all(project_path, run_id, stop_on_failure=True, suite_types=None):
            """Mock run_all that returns results for requested suite types."""
            suite_types = suite_types or [
                TestSuiteType.UNIT,
                TestSuiteType.INTEGRATION,
                TestSuiteType.E2E,
            ]
            results = []
            for suite_type in suite_types:
                results.append(
                    TestSuiteExecution(
                        execution_id=uuid4(),
                        run_id=run_id,
                        suite_type=suite_type,
                        status=TestRunnerStatus.COMPLETED,
                        started_at=datetime.utcnow(),
                        completed_at=datetime.utcnow(),
                        total_tests=5,
                        passed=5,
                        failed=0,
                        skipped=0,
                        errors=0,
                    )
                )
            return results

        orchestrator.run_all = MagicMock(side_effect=mock_run_all)
        return orchestrator

    @pytest.fixture
    def integration(self, mock_orchestrator):
        """Create test pipeline integration with mocks."""
        with patch("src.testing.pipeline_integration.get_history_tracker"), \
             patch("src.testing.pipeline_integration.get_flaky_detector"), \
             patch("src.testing.pipeline_integration.get_report_aggregator"), \
             patch("src.testing.pipeline_integration.get_framework_detector"):
            integration = TestPipelineIntegration(
                orchestrator=mock_orchestrator,
            )
            return integration

    @pytest.mark.asyncio
    async def test_unit_tests_handler_runs_unit_tests(
        self, integration, mock_orchestrator
    ):
        """Test that unit_tests_handler runs unit tests."""
        from src.core.pipeline import PipelineContext, PipelineConfig
        from src.core.models import ValidationRun

        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        context = PipelineContext(
            run=run,
            config=PipelineConfig(),
            project_path="/test/project",
        )

        result = await integration.unit_tests_handler(context)

        assert "status" in result
        mock_orchestrator.run_all.assert_called()
        # Verify unit tests were requested
        call_kwargs = mock_orchestrator.run_all.call_args[1]
        assert TestSuiteType.UNIT in call_kwargs.get("suite_types", [])

    @pytest.mark.asyncio
    async def test_integration_tests_handler_runs_integration_tests(
        self, integration, mock_orchestrator
    ):
        """Test that integration_tests_handler runs integration tests."""
        from src.core.pipeline import PipelineContext, PipelineConfig
        from src.core.models import ValidationRun

        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        context = PipelineContext(
            run=run,
            config=PipelineConfig(),
            project_path="/test/project",
        )

        result = await integration.integration_tests_handler(context)

        assert "status" in result
        call_kwargs = mock_orchestrator.run_all.call_args[1]
        assert TestSuiteType.INTEGRATION in call_kwargs.get("suite_types", [])

    @pytest.mark.asyncio
    async def test_e2e_tests_handler_runs_e2e_tests(
        self, integration, mock_orchestrator
    ):
        """Test that e2e_tests_handler runs E2E tests."""
        from src.core.pipeline import PipelineContext, PipelineConfig
        from src.core.models import ValidationRun

        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        context = PipelineContext(
            run=run,
            config=PipelineConfig(),
            project_path="/test/project",
        )

        result = await integration.e2e_tests_handler(context)

        assert "status" in result
        call_kwargs = mock_orchestrator.run_all.call_args[1]
        assert TestSuiteType.E2E in call_kwargs.get("suite_types", [])

    @pytest.mark.asyncio
    async def test_all_tests_handler_runs_all_suites(
        self, integration, mock_orchestrator
    ):
        """Test that all_tests_handler runs all test suites."""
        from src.core.pipeline import PipelineContext, PipelineConfig
        from src.core.models import ValidationRun

        run = ValidationRun(
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )
        context = PipelineContext(
            run=run,
            config=PipelineConfig(),
            project_path="/test/project",
        )

        result = await integration.all_tests_handler(context)

        assert "status" in result
        assert "results" in result
        assert "report" in result

    def test_register_handlers_registers_all_stages(self, integration):
        """Test that register_handlers registers all test stages."""
        pipeline = MagicMock(spec=ValidationPipeline)

        integration.register_handlers(pipeline)

        # Verify all test stages are registered
        assert pipeline.register_stage_handler.call_count == 3
        registered_stages = [
            call[0][0] for call in pipeline.register_stage_handler.call_args_list
        ]
        assert PipelineStage.UNIT_TESTS in registered_stages
        assert PipelineStage.INTEGRATION_TESTS in registered_stages
        assert PipelineStage.E2E_TESTS in registered_stages


class TestPipelineWithTestExecution:
    """Integration tests for full pipeline with test execution."""

    @pytest.fixture
    def mock_storage(self):
        """Create mock storage."""
        storage = MagicMock()
        storage.save_run = MagicMock()
        return storage

    @pytest.mark.asyncio
    async def test_full_pipeline_with_tests_only(self, mock_storage):
        """Test running a full pipeline with only test stages."""
        with patch("src.core.pipeline.get_storage", return_value=mock_storage):
            pipeline = ValidationPipeline()

            # Register mock test handlers
            async def mock_test_handler(context):
                return {"status": CheckStatus.PASSED}

            pipeline.register_stage_handler(
                PipelineStage.UNIT_TESTS, mock_test_handler
            )
            pipeline.register_stage_handler(
                PipelineStage.INTEGRATION_TESTS, mock_test_handler
            )
            pipeline.register_stage_handler(
                PipelineStage.E2E_TESTS, mock_test_handler
            )

            config = PipelineConfig(
                run_quality=False,
                run_tests=True,
                run_deployment=False,
            )

            run = await pipeline.run(
                project_path="/test/project",
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                config=config,
            )

            assert run.status == ValidationStatus.PASSED

    @pytest.mark.asyncio
    async def test_pipeline_stops_on_test_failure_with_fail_fast(self, mock_storage):
        """Test that pipeline stops on test failure when fail_fast=True."""
        with patch("src.core.pipeline.get_storage", return_value=mock_storage):
            pipeline = ValidationPipeline()

            call_count = {"unit": 0, "integration": 0, "e2e": 0}

            async def failing_unit_handler(context):
                call_count["unit"] += 1
                return {"status": CheckStatus.FAILED}

            async def passing_handler(context, stage_name):
                call_count[stage_name] += 1
                return {"status": CheckStatus.PASSED}

            pipeline.register_stage_handler(
                PipelineStage.UNIT_TESTS, failing_unit_handler
            )
            pipeline.register_stage_handler(
                PipelineStage.INTEGRATION_TESTS,
                lambda ctx: passing_handler(ctx, "integration"),
            )
            pipeline.register_stage_handler(
                PipelineStage.E2E_TESTS,
                lambda ctx: passing_handler(ctx, "e2e"),
            )

            config = PipelineConfig(
                run_quality=False,
                run_tests=True,
                run_deployment=False,
                fail_fast=True,
            )

            run = await pipeline.run(
                project_path="/test/project",
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                config=config,
            )

            assert run.status == ValidationStatus.FAILED
            assert call_count["unit"] == 1
            # Integration and E2E should not run due to fail_fast
            assert call_count["integration"] == 0
            assert call_count["e2e"] == 0


class TestSetupTestPipeline:
    """Tests for setup_test_pipeline function."""

    def test_setup_registers_handlers(self):
        """Test that setup_test_pipeline registers handlers."""
        with patch("src.testing.pipeline_integration.get_pipeline") as mock_get_pipeline, \
             patch("src.testing.pipeline_integration.get_test_pipeline_integration") as mock_get_integration:
            mock_pipeline = MagicMock()
            mock_get_pipeline.return_value = mock_pipeline

            mock_integration = MagicMock()
            mock_get_integration.return_value = mock_integration

            setup_test_pipeline()

            mock_integration.register_handlers.assert_called_once()
