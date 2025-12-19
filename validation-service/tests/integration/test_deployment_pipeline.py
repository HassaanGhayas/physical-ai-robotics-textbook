"""Integration tests for deployment validation pipeline.

Tests the integration between deployment validators and the validation pipeline.
"""

import pytest
import asyncio
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch, AsyncMock
from uuid import uuid4

from src.core.models import (
    TriggerSource,
    ValidationStatus,
    CheckStatus,
)
from src.core.pipeline import (
    ValidationPipeline,
    PipelineConfig,
    PipelineStage,
)
from src.deployment.config_validator import ConfigValidator
from src.deployment.dependency_checker import DependencyChecker
from src.deployment.security_scanner import SecurityScanner
from src.deployment.compatibility import CompatibilityChecker
from src.deployment.pipeline_integration import (
    DeploymentPipelineIntegration,
    get_deployment_pipeline_integration,
    setup_deployment_pipeline,
)
from src.deployment.models import ConfigCheckStatus


class TestDeploymentPipelineIntegration:
    """Integration tests for deployment pipeline integration."""

    @pytest.fixture
    def mock_config_validator(self):
        """Create mock config validator."""
        validator = MagicMock(spec=ConfigValidator)
        from src.deployment.models import ConfigurationValidationResult
        validator.validate.return_value = ConfigurationValidationResult(
            status=ConfigCheckStatus.VALID,
            checks=[],
            missing_required=[],
            invalid_values=[],
            secret_risks=[],
        )
        return validator

    @pytest.fixture
    def mock_dependency_checker(self):
        """Create mock dependency checker."""
        checker = MagicMock(spec=DependencyChecker)
        from src.deployment.models import DependencyValidationResult
        checker.check.return_value = DependencyValidationResult(
            status=ConfigCheckStatus.VALID,
            total_dependencies=10,
            conflicts=[],
            outdated_packages=[],
            security_vulnerabilities=0,
        )
        return checker

    @pytest.fixture
    def mock_security_scanner(self):
        """Create mock security scanner."""
        scanner = MagicMock(spec=SecurityScanner)
        from src.deployment.models import ConfigurationValidationResult
        scanner.scan.return_value = ConfigurationValidationResult(
            status=ConfigCheckStatus.VALID,
            checks=[],
            secret_risks=[],
        )
        return scanner

    @pytest.fixture
    def mock_compatibility_checker(self):
        """Create mock compatibility checker."""
        checker = MagicMock(spec=CompatibilityChecker)
        from src.deployment.models import CompatibilityValidationResult
        checker.check.return_value = CompatibilityValidationResult(
            status=ConfigCheckStatus.VALID,
            resource_checks=[],
            platform_checks=[],
            service_checks=[],
        )
        return checker

    @pytest.fixture
    def integration(
        self,
        mock_config_validator,
        mock_dependency_checker,
        mock_security_scanner,
        mock_compatibility_checker,
    ):
        """Create deployment pipeline integration with mocks."""
        return DeploymentPipelineIntegration(
            config_validator=mock_config_validator,
            dependency_checker=mock_dependency_checker,
            security_scanner=mock_security_scanner,
            compatibility_checker=mock_compatibility_checker,
        )

    @pytest.mark.asyncio
    async def test_deployment_handler_all_passing(self, integration):
        """Test deployment handler when all checks pass."""
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

        result = await integration.deployment_handler(context)

        assert result["status"] == CheckStatus.PASSED
        assert result["is_deployable"] is True
        assert run.deployment_checklist is not None

    @pytest.mark.asyncio
    async def test_deployment_handler_with_secrets_detected(
        self, integration, mock_config_validator
    ):
        """Test deployment handler when secrets are detected."""
        from src.deployment.models import ConfigurationValidationResult, SecretRisk, SeverityLevel

        mock_config_validator.validate.return_value = ConfigurationValidationResult(
            status=ConfigCheckStatus.INVALID,
            checks=[],
            missing_required=[],
            invalid_values=[],
            secret_risks=[
                SecretRisk(
                    file_path="config.py",
                    line_number=10,
                    secret_type="API Key",
                    severity=SeverityLevel.HIGH,
                    pattern_matched="api_key",
                    recommendation="Move to env var",
                ),
            ],
        )

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

        result = await integration.deployment_handler(context)

        assert result["status"] == CheckStatus.FAILED
        assert result["is_deployable"] is False

    @pytest.mark.asyncio
    async def test_deployment_handler_with_vulnerabilities(
        self, integration, mock_dependency_checker
    ):
        """Test deployment handler when vulnerabilities are detected."""
        from src.deployment.models import DependencyValidationResult

        mock_dependency_checker.check.return_value = DependencyValidationResult(
            status=ConfigCheckStatus.INVALID,
            total_dependencies=10,
            conflicts=[],
            outdated_packages=[],
            security_vulnerabilities=5,
        )

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

        result = await integration.deployment_handler(context)

        assert result["status"] == CheckStatus.FAILED
        assert result["is_deployable"] is False

    def test_register_handler_registers_deployment_stage(self, integration):
        """Test that register_handler registers the deployment stage."""
        pipeline = MagicMock(spec=ValidationPipeline)

        integration.register_handler(pipeline)

        pipeline.register_stage_handler.assert_called_once_with(
            PipelineStage.DEPLOYMENT,
            integration.deployment_handler,
        )


class TestPipelineWithDeploymentValidation:
    """Integration tests for full pipeline with deployment validation."""

    @pytest.fixture
    def mock_storage(self):
        """Create mock storage."""
        storage = MagicMock()
        storage.save_run = MagicMock()
        return storage

    @pytest.mark.asyncio
    async def test_full_pipeline_with_deployment_only(self, mock_storage):
        """Test running a full pipeline with only deployment stage."""
        with patch("src.core.pipeline.get_storage", return_value=mock_storage):
            pipeline = ValidationPipeline()

            # Register mock deployment handler
            async def mock_deployment_handler(context):
                return {"status": CheckStatus.PASSED}

            pipeline.register_stage_handler(
                PipelineStage.DEPLOYMENT, mock_deployment_handler
            )

            config = PipelineConfig(
                run_quality=False,
                run_tests=False,
                run_deployment=True,
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
    async def test_pipeline_fails_on_deployment_failure(self, mock_storage):
        """Test that pipeline fails when deployment validation fails."""
        with patch("src.core.pipeline.get_storage", return_value=mock_storage):
            pipeline = ValidationPipeline()

            async def failing_deployment_handler(context):
                return {"status": CheckStatus.FAILED}

            pipeline.register_stage_handler(
                PipelineStage.DEPLOYMENT, failing_deployment_handler
            )

            config = PipelineConfig(
                run_quality=False,
                run_tests=False,
                run_deployment=True,
            )

            run = await pipeline.run(
                project_path="/test/project",
                repository="test-repo",
                branch="main",
                commit_sha="abc123",
                config=config,
            )

            assert run.status == ValidationStatus.FAILED


class TestSetupDeploymentPipeline:
    """Tests for setup_deployment_pipeline function."""

    def test_setup_registers_handler(self):
        """Test that setup_deployment_pipeline registers handler."""
        with patch("src.deployment.pipeline_integration.get_pipeline") as mock_get_pipeline, \
             patch("src.deployment.pipeline_integration.get_deployment_pipeline_integration") as mock_get_integration:
            mock_pipeline = MagicMock()
            mock_get_pipeline.return_value = mock_pipeline

            mock_integration = MagicMock()
            mock_get_integration.return_value = mock_integration

            setup_deployment_pipeline()

            mock_integration.register_handler.assert_called_once()
