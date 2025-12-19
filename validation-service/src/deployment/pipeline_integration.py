"""Integration of deployment validation with the validation pipeline.

This module provides the pipeline stage handler for deployment validation,
integrating the deployment checkers with the validation pipeline.
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    CheckStatus,
    DeploymentChecklist,
    ConfigurationCheck,
    DependencyCheck,
    CompatibilityCheck,
)
from src.core.pipeline import (
    PipelineStage,
    PipelineContext,
    ValidationPipeline,
    get_pipeline,
)
from src.deployment.config_validator import ConfigValidator, get_config_validator
from src.deployment.dependency_checker import DependencyChecker, get_dependency_checker
from src.deployment.security_scanner import SecurityScanner, get_security_scanner
from src.deployment.compatibility import CompatibilityChecker, get_compatibility_checker
from src.deployment.models import ConfigCheckStatus


logger = logging.getLogger(__name__)


class DeploymentPipelineIntegration:
    """Integrates deployment validation with the validation pipeline."""

    def __init__(
        self,
        config_validator: Optional[ConfigValidator] = None,
        dependency_checker: Optional[DependencyChecker] = None,
        security_scanner: Optional[SecurityScanner] = None,
        compatibility_checker: Optional[CompatibilityChecker] = None,
    ):
        """Initialize the deployment pipeline integration.

        Args:
            config_validator: Configuration validator instance
            dependency_checker: Dependency checker instance
            security_scanner: Security scanner instance
            compatibility_checker: Compatibility checker instance
        """
        self.settings = get_settings()
        self.config_validator = config_validator or get_config_validator()
        self.dependency_checker = dependency_checker or get_dependency_checker()
        self.security_scanner = security_scanner or get_security_scanner()
        self.compatibility_checker = compatibility_checker or get_compatibility_checker()

    async def deployment_handler(self, context: PipelineContext) -> dict:
        """Pipeline stage handler for deployment validation.

        Args:
            context: Pipeline execution context

        Returns:
            Dictionary with status and results
        """
        project_path = Path(context.project_path)
        run = context.run

        logger.info(f"Running deployment validation for {project_path}")

        # Create deployment checklist
        checklist = DeploymentChecklist(run_id=run.run_id)

        # Validate configuration
        config_result = self.config_validator.validate(project_path)
        checklist.configuration = ConfigurationCheck(
            status=self._convert_status(config_result.status),
            required_variables=len(config_result.checks),
            missing_variables=len(config_result.missing_required),
            invalid_values=len(config_result.invalid_values),
            secret_exposures=len(config_result.secret_risks),
            config_checks=[{
                "name": c.name,
                "status": c.status.value,
                "message": c.error_message,
            } for c in config_result.checks[:10]],  # Limit to first 10
            secret_risks=[{
                "file": s.file_path,
                "line": s.line_number,
                "type": s.secret_type,
                "severity": s.severity.value,
            } for s in config_result.secret_risks[:10]],
        )

        # Check dependencies
        dep_result = self.dependency_checker.check(project_path)
        checklist.dependencies = DependencyCheck(
            status=self._convert_status(dep_result.status),
            total_packages=dep_result.total_dependencies,
            vulnerable_packages=dep_result.security_vulnerabilities,
            outdated_packages=len(dep_result.outdated_packages),
            conflicts=[{
                "package": c.package,
                "type": c.conflict_type,
                "resolution": c.resolution,
            } for c in dep_result.conflicts],
            outdated=[{
                "name": o.name,
                "current": o.current_version,
                "latest": o.latest_version,
            } for o in dep_result.outdated_packages[:10]],
        )

        # Run security scan
        security_result = self.security_scanner.scan(project_path)

        # Check compatibility
        compat_result = self.compatibility_checker.check(project_path)
        checklist.compatibility = CompatibilityCheck(
            status=self._convert_status(compat_result.status),
            resource_checks=[{
                "type": r.resource_type,
                "required": r.required,
                "available": r.available,
                "status": r.status.value,
            } for r in compat_result.resource_checks],
            platform_checks=[{
                "platform": p.platform,
                "required": p.required_version,
                "detected": p.detected_version,
                "compatible": p.is_compatible,
            } for p in compat_result.platform_checks],
            service_checks=[{
                "service": s.service_name,
                "endpoint": s.endpoint,
                "status": s.status.value,
                "latency_ms": s.latency_ms,
            } for s in compat_result.service_checks],
        )

        # Determine overall status
        all_passed = all([
            config_result.status == ConfigCheckStatus.VALID,
            dep_result.status == ConfigCheckStatus.VALID,
            compat_result.status == ConfigCheckStatus.VALID,
        ])

        has_critical = (
            len(config_result.secret_risks) > 0 or
            dep_result.security_vulnerabilities > 0 or
            any(not p.is_compatible for p in compat_result.platform_checks)
        )

        if has_critical:
            checklist.status = CheckStatus.FAILED
        elif not all_passed:
            checklist.status = CheckStatus.WARNING
        else:
            checklist.status = CheckStatus.PASSED

        # Attach to run
        run.deployment_checklist = checklist

        return {
            "status": checklist.status,
            "checklist": checklist,
            "is_deployable": checklist.status != CheckStatus.FAILED,
        }

    def _convert_status(self, status: ConfigCheckStatus) -> CheckStatus:
        """Convert ConfigCheckStatus to CheckStatus.

        Args:
            status: Configuration check status

        Returns:
            Pipeline check status
        """
        if status == ConfigCheckStatus.VALID:
            return CheckStatus.PASSED
        elif status == ConfigCheckStatus.WARNING:
            return CheckStatus.WARNING
        else:
            return CheckStatus.FAILED

    def register_handler(self, pipeline: Optional[ValidationPipeline] = None) -> None:
        """Register deployment handler with the pipeline.

        Args:
            pipeline: Pipeline to register with (uses default if None)
        """
        pipeline = pipeline or get_pipeline()

        pipeline.register_stage_handler(
            PipelineStage.DEPLOYMENT,
            self.deployment_handler,
        )

        logger.info("Deployment pipeline handler registered")


# Module-level singleton
_integration: Optional[DeploymentPipelineIntegration] = None


def get_deployment_pipeline_integration() -> DeploymentPipelineIntegration:
    """Get deployment pipeline integration instance.

    Returns:
        DeploymentPipelineIntegration singleton
    """
    global _integration
    if _integration is None:
        _integration = DeploymentPipelineIntegration()
    return _integration


def setup_deployment_pipeline() -> None:
    """Set up deployment validation in the validation pipeline.

    This function should be called during application startup to
    register deployment handler with the pipeline.
    """
    integration = get_deployment_pipeline_integration()
    integration.register_handler()
    logger.info("Deployment pipeline integration complete")
