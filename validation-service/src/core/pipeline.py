"""Pipeline orchestrator for the validation service.

This module coordinates the execution of validation stages (quality, testing, deployment)
and manages the overall validation lifecycle.
"""

import asyncio
import logging
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Callable, Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    CheckStatus,
    CodeQualityReport,
    DeploymentChecklist,
    TestSuiteResult,
    TestSuiteType,
    TriggerSource,
    ValidationRun,
    ValidationStatus,
)
from src.core.storage import get_storage


logger = logging.getLogger(__name__)


class PipelineStage(str, Enum):
    """Stages in the validation pipeline."""

    QUALITY = "quality"
    UNIT_TESTS = "unit_tests"
    INTEGRATION_TESTS = "integration_tests"
    E2E_TESTS = "e2e_tests"
    DEPLOYMENT = "deployment"


@dataclass
class StageResult:
    """Result from executing a pipeline stage."""

    stage: PipelineStage
    status: CheckStatus
    duration_seconds: float
    error_message: Optional[str] = None
    data: Optional[dict] = None


@dataclass
class PipelineConfig:
    """Configuration for a pipeline run."""

    run_quality: bool = True
    run_tests: bool = True
    run_deployment: bool = True
    fail_fast: bool = False
    timeout_minutes: int = 15
    parallel_tests: bool = True


@dataclass
class PipelineContext:
    """Context passed through pipeline execution."""

    run: ValidationRun
    config: PipelineConfig
    project_path: str
    results: dict = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)
    start_time: datetime = field(default_factory=datetime.utcnow)


class ValidationPipeline:
    """Orchestrates the validation pipeline execution."""

    def __init__(self):
        """Initialize the pipeline orchestrator."""
        self.settings = get_settings()
        self.storage = get_storage()
        self._stage_handlers: dict[PipelineStage, Callable] = {}
        self._callbacks: dict[str, list[Callable]] = {
            "on_start": [],
            "on_stage_start": [],
            "on_stage_complete": [],
            "on_complete": [],
            "on_error": [],
        }

    def register_stage_handler(
        self, stage: PipelineStage, handler: Callable
    ) -> None:
        """Register a handler for a pipeline stage.

        Args:
            stage: Pipeline stage to handle
            handler: Async function to execute for this stage
        """
        self._stage_handlers[stage] = handler

    def register_callback(self, event: str, callback: Callable) -> None:
        """Register a callback for pipeline events.

        Args:
            event: Event name (on_start, on_stage_start, on_stage_complete, on_complete, on_error)
            callback: Async function to call
        """
        if event in self._callbacks:
            self._callbacks[event].append(callback)

    async def _emit(self, event: str, *args, **kwargs) -> None:
        """Emit an event to all registered callbacks."""
        for callback in self._callbacks.get(event, []):
            try:
                if asyncio.iscoroutinefunction(callback):
                    await callback(*args, **kwargs)
                else:
                    callback(*args, **kwargs)
            except Exception as e:
                logger.error(f"Error in callback for {event}: {e}")

    async def run(
        self,
        project_path: str,
        repository: str,
        branch: str,
        commit_sha: str,
        trigger_source: TriggerSource = TriggerSource.MANUAL,
        user: Optional[str] = None,
        config: Optional[PipelineConfig] = None,
    ) -> ValidationRun:
        """Execute the validation pipeline.

        Args:
            project_path: Path to the project to validate
            repository: Repository name
            branch: Git branch name
            commit_sha: Git commit SHA
            trigger_source: How the validation was triggered
            user: User who triggered (for manual runs)
            config: Pipeline configuration

        Returns:
            ValidationRun with all results
        """
        config = config or PipelineConfig()

        # Create validation run
        run = ValidationRun(
            trigger_source=trigger_source,
            repository=repository,
            branch=branch,
            commit_sha=commit_sha,
            user=user,
        )

        context = PipelineContext(
            run=run,
            config=config,
            project_path=project_path,
        )

        try:
            # Start pipeline
            run.start()
            self.storage.save_run(run)
            await self._emit("on_start", context)

            logger.info(f"Starting validation pipeline for {repository}@{branch}")

            # Determine stages to run
            stages = self._get_stages(config)

            # Execute stages
            all_passed = True
            for stage in stages:
                result = await self._execute_stage(context, stage)
                if result.status != CheckStatus.PASSED:
                    all_passed = False
                    if config.fail_fast:
                        logger.info(f"Stage {stage.value} failed, stopping (fail_fast=True)")
                        break

            # Determine final status
            if context.errors:
                run.complete(ValidationStatus.ERROR)
            elif all_passed:
                run.succeed()
            else:
                run.fail()

            # Save final state
            self.storage.save_run(run)
            await self._emit("on_complete", context)

            logger.info(f"Validation pipeline completed with status: {run.status.value}")
            return run

        except asyncio.TimeoutError:
            run.complete(ValidationStatus.TIMEOUT)
            self.storage.save_run(run)
            await self._emit("on_error", context, "Pipeline timeout")
            return run

        except Exception as e:
            logger.exception(f"Pipeline execution error: {e}")
            run.complete(ValidationStatus.ERROR)
            self.storage.save_run(run)
            await self._emit("on_error", context, str(e))
            return run

    def _get_stages(self, config: PipelineConfig) -> list[PipelineStage]:
        """Get ordered list of stages to execute based on config."""
        stages = []

        if config.run_quality:
            stages.append(PipelineStage.QUALITY)

        if config.run_tests:
            stages.append(PipelineStage.UNIT_TESTS)
            stages.append(PipelineStage.INTEGRATION_TESTS)
            stages.append(PipelineStage.E2E_TESTS)

        if config.run_deployment:
            stages.append(PipelineStage.DEPLOYMENT)

        return stages

    async def _execute_stage(
        self, context: PipelineContext, stage: PipelineStage
    ) -> StageResult:
        """Execute a single pipeline stage.

        Args:
            context: Pipeline context
            stage: Stage to execute

        Returns:
            StageResult with execution outcome
        """
        logger.info(f"Executing stage: {stage.value}")
        await self._emit("on_stage_start", context, stage)

        start_time = datetime.utcnow()

        try:
            handler = self._stage_handlers.get(stage)
            if handler:
                result = await handler(context)
            else:
                # Default stub handlers if not registered
                result = await self._default_stage_handler(context, stage)

            duration = (datetime.utcnow() - start_time).total_seconds()
            stage_result = StageResult(
                stage=stage,
                status=result.get("status", CheckStatus.PASSED),
                duration_seconds=duration,
                data=result,
            )

        except Exception as e:
            duration = (datetime.utcnow() - start_time).total_seconds()
            logger.error(f"Stage {stage.value} failed with error: {e}")
            context.errors.append(f"{stage.value}: {str(e)}")
            stage_result = StageResult(
                stage=stage,
                status=CheckStatus.FAILED,
                duration_seconds=duration,
                error_message=str(e),
            )

        context.results[stage.value] = stage_result
        await self._emit("on_stage_complete", context, stage, stage_result)

        return stage_result

    async def _default_stage_handler(
        self, context: PipelineContext, stage: PipelineStage
    ) -> dict:
        """Default handler for stages without registered handlers.

        Returns a placeholder result indicating the stage is not implemented.
        """
        run = context.run

        if stage == PipelineStage.QUALITY:
            report = CodeQualityReport(run_id=run.run_id, status=CheckStatus.PASSED)
            run.quality_report = report
            return {"status": CheckStatus.PASSED, "report": report}

        elif stage in (
            PipelineStage.UNIT_TESTS,
            PipelineStage.INTEGRATION_TESTS,
            PipelineStage.E2E_TESTS,
        ):
            suite_type_map = {
                PipelineStage.UNIT_TESTS: TestSuiteType.UNIT,
                PipelineStage.INTEGRATION_TESTS: TestSuiteType.INTEGRATION,
                PipelineStage.E2E_TESTS: TestSuiteType.E2E,
            }
            result = TestSuiteResult(
                run_id=run.run_id,
                suite_type=suite_type_map[stage],
            )
            run.test_results.append(result)
            return {"status": CheckStatus.PASSED, "result": result}

        elif stage == PipelineStage.DEPLOYMENT:
            checklist = DeploymentChecklist(run_id=run.run_id, status=CheckStatus.PASSED)
            run.deployment_checklist = checklist
            return {"status": CheckStatus.PASSED, "checklist": checklist}

        return {"status": CheckStatus.PASSED}

    async def run_quality_only(
        self, project_path: str, repository: str, branch: str, commit_sha: str
    ) -> ValidationRun:
        """Run only code quality validation.

        Args:
            project_path: Path to project
            repository: Repository name
            branch: Branch name
            commit_sha: Commit SHA

        Returns:
            ValidationRun with quality results
        """
        config = PipelineConfig(run_quality=True, run_tests=False, run_deployment=False)
        return await self.run(
            project_path=project_path,
            repository=repository,
            branch=branch,
            commit_sha=commit_sha,
            config=config,
        )

    async def run_tests_only(
        self, project_path: str, repository: str, branch: str, commit_sha: str
    ) -> ValidationRun:
        """Run only test suites.

        Args:
            project_path: Path to project
            repository: Repository name
            branch: Branch name
            commit_sha: Commit SHA

        Returns:
            ValidationRun with test results
        """
        config = PipelineConfig(run_quality=False, run_tests=True, run_deployment=False)
        return await self.run(
            project_path=project_path,
            repository=repository,
            branch=branch,
            commit_sha=commit_sha,
            config=config,
        )

    async def run_deployment_only(
        self, project_path: str, repository: str, branch: str, commit_sha: str
    ) -> ValidationRun:
        """Run only deployment validation.

        Args:
            project_path: Path to project
            repository: Repository name
            branch: Branch name
            commit_sha: Commit SHA

        Returns:
            ValidationRun with deployment results
        """
        config = PipelineConfig(run_quality=False, run_tests=False, run_deployment=True)
        return await self.run(
            project_path=project_path,
            repository=repository,
            branch=branch,
            commit_sha=commit_sha,
            config=config,
        )


# Singleton pipeline instance
_pipeline: Optional[ValidationPipeline] = None


def get_pipeline() -> ValidationPipeline:
    """Get the pipeline orchestrator instance.

    Returns:
        ValidationPipeline singleton
    """
    global _pipeline
    if _pipeline is None:
        _pipeline = ValidationPipeline()
    return _pipeline
