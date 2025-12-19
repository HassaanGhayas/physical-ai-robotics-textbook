"""Integration of test execution with the validation pipeline.

This module provides the pipeline stage handlers for test execution,
integrating the test orchestrator with the validation pipeline.
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    CheckStatus,
    TestStatus,
    TestSuiteType,
    TestSuiteResult,
)
from src.core.pipeline import (
    PipelineStage,
    PipelineContext,
    ValidationPipeline,
    get_pipeline,
)
from src.testing.orchestrator import TestOrchestrator, get_test_orchestrator
from src.testing.history import TestHistoryTracker, get_history_tracker
from src.testing.flaky_detector import FlakyTestDetector, get_flaky_detector
from src.testing.report_aggregator import TestReportAggregator, get_report_aggregator
from src.testing.retry_handler import (
    RetryHandler,
    RetryConfig,
    RetryStrategy,
    SelectiveRetryHandler,
    get_selective_retry_handler,
)
from src.testing.framework_detector import TestFrameworkDetector, get_framework_detector


logger = logging.getLogger(__name__)


class TestPipelineIntegration:
    """Integrates test execution with the validation pipeline."""

    def __init__(
        self,
        orchestrator: Optional[TestOrchestrator] = None,
        history_tracker: Optional[TestHistoryTracker] = None,
        flaky_detector: Optional[FlakyTestDetector] = None,
        report_aggregator: Optional[TestReportAggregator] = None,
        framework_detector: Optional[TestFrameworkDetector] = None,
    ):
        """Initialize the test pipeline integration.

        Args:
            orchestrator: Test orchestrator instance
            history_tracker: History tracker instance
            flaky_detector: Flaky test detector instance
            report_aggregator: Report aggregator instance
            framework_detector: Framework detector instance
        """
        self.settings = get_settings()
        self.orchestrator = orchestrator or get_test_orchestrator()
        self.history_tracker = history_tracker or get_history_tracker()
        self.flaky_detector = flaky_detector or get_flaky_detector()
        self.report_aggregator = report_aggregator or get_report_aggregator()
        self.framework_detector = framework_detector or get_framework_detector()

        # Configure retry handler with known flaky tests
        flaky_tests = self.history_tracker.get_flaky_tests()
        flaky_test_ids = {test.test_id for test in flaky_tests}
        self.retry_handler = get_selective_retry_handler(
            config=RetryConfig(
                max_retries=3,
                strategy=RetryStrategy.EXPONENTIAL,
                base_delay_seconds=1.0,
            ),
            flaky_test_ids=flaky_test_ids,
        )

    async def unit_tests_handler(self, context: PipelineContext) -> dict:
        """Pipeline stage handler for unit tests.

        Args:
            context: Pipeline execution context

        Returns:
            Dictionary with status and results
        """
        return await self._run_test_suite(
            context=context,
            suite_type=TestSuiteType.UNIT,
        )

    async def integration_tests_handler(self, context: PipelineContext) -> dict:
        """Pipeline stage handler for integration tests.

        Args:
            context: Pipeline execution context

        Returns:
            Dictionary with status and results
        """
        return await self._run_test_suite(
            context=context,
            suite_type=TestSuiteType.INTEGRATION,
        )

    async def e2e_tests_handler(self, context: PipelineContext) -> dict:
        """Pipeline stage handler for E2E tests.

        Args:
            context: Pipeline execution context

        Returns:
            Dictionary with status and results
        """
        return await self._run_test_suite(
            context=context,
            suite_type=TestSuiteType.E2E,
        )

    async def _run_test_suite(
        self,
        context: PipelineContext,
        suite_type: TestSuiteType,
    ) -> dict:
        """Run a specific test suite.

        Args:
            context: Pipeline context
            suite_type: Type of test suite to run

        Returns:
            Dictionary with status and results
        """
        project_path = Path(context.project_path)
        run = context.run

        logger.info(f"Running {suite_type.value} tests for {project_path}")

        # Detect framework
        test_config = self.framework_detector.detect(project_path)
        logger.debug(f"Detected test configuration: {test_config}")

        # Run the specific suite
        results = self.orchestrator.run_all(
            project_path=project_path,
            run_id=run.run_id,
            stop_on_failure=context.config.fail_fast,
            suite_types=[suite_type],
        )

        if not results:
            # No tests found for this suite
            result = TestSuiteResult(
                run_id=run.run_id,
                suite_type=suite_type,
                status=TestStatus.SKIPPED,
                started_at=datetime.utcnow(),
                completed_at=datetime.utcnow(),
            )
            run.test_results.append(result)
            return {"status": CheckStatus.PASSED, "result": result, "skipped": True}

        result = results[0]

        # Record execution history for each test case
        for test_case_result in result.test_cases:
            self.history_tracker.record_execution(
                test_id=test_case_result.test_id,
                test_name=test_case_result.test_name,
                test_type=suite_type,
                file_path="",  # TODO: Get from test case
                run_id=run.run_id,
                status=test_case_result.status,
                duration_seconds=test_case_result.duration_seconds,
                commit_sha=run.commit_sha,
            )

        # Check for flaky tests
        flaky_detected = []
        for test_case_result in result.test_cases:
            test_history = self.history_tracker.get_test_history(
                test_case_result.test_id
            )
            if test_history:
                flaky_record = self.flaky_detector.analyze_test(test_history)
                if flaky_record:
                    flaky_detected.append(flaky_record.test_id)
                    # Add to retry handler's known flaky set
                    self.retry_handler.add_flaky_test(flaky_record.test_id)

        result.flaky_tests_detected = flaky_detected

        # Attach to run
        run.test_results.append(result)

        # Determine status
        if result.status == TestStatus.PASSED:
            status = CheckStatus.PASSED
        elif result.status == TestStatus.SKIPPED:
            status = CheckStatus.WARNING
        else:
            status = CheckStatus.FAILED

        return {
            "status": status,
            "result": result,
            "flaky_tests": flaky_detected,
        }

    async def all_tests_handler(self, context: PipelineContext) -> dict:
        """Pipeline handler for running all tests in sequence.

        This is an alternative to running each suite as a separate stage.

        Args:
            context: Pipeline execution context

        Returns:
            Dictionary with status and aggregated results
        """
        project_path = Path(context.project_path)
        run = context.run

        # Run all test suites
        results = self.orchestrator.run_all(
            project_path=project_path,
            run_id=run.run_id,
            stop_on_failure=context.config.fail_fast,
        )

        # Attach to run
        run.test_results = results

        # Generate aggregated report
        report = self.report_aggregator.aggregate(results)

        # Determine overall status
        all_passed = all(r.status == TestStatus.PASSED for r in results)
        has_failures = any(r.status == TestStatus.FAILED for r in results)
        has_errors = any(r.status == TestStatus.ERROR for r in results)

        if has_errors:
            status = CheckStatus.FAILED
        elif has_failures:
            status = CheckStatus.FAILED
        elif all_passed:
            status = CheckStatus.PASSED
        else:
            status = CheckStatus.WARNING

        return {
            "status": status,
            "results": results,
            "report": report,
        }

    def register_handlers(self, pipeline: Optional[ValidationPipeline] = None) -> None:
        """Register test handlers with the pipeline.

        Args:
            pipeline: Pipeline to register with (uses default if None)
        """
        pipeline = pipeline or get_pipeline()

        pipeline.register_stage_handler(
            PipelineStage.UNIT_TESTS,
            self.unit_tests_handler,
        )
        pipeline.register_stage_handler(
            PipelineStage.INTEGRATION_TESTS,
            self.integration_tests_handler,
        )
        pipeline.register_stage_handler(
            PipelineStage.E2E_TESTS,
            self.e2e_tests_handler,
        )

        logger.info("Test pipeline handlers registered")


# Module-level singleton
_integration: Optional[TestPipelineIntegration] = None


def get_test_pipeline_integration() -> TestPipelineIntegration:
    """Get test pipeline integration instance.

    Returns:
        TestPipelineIntegration singleton
    """
    global _integration
    if _integration is None:
        _integration = TestPipelineIntegration()
    return _integration


def setup_test_pipeline() -> None:
    """Set up test execution in the validation pipeline.

    This function should be called during application startup to
    register test handlers with the pipeline.
    """
    integration = get_test_pipeline_integration()
    integration.register_handlers()
    logger.info("Test pipeline integration complete")
