"""Testing module - Test suite execution and orchestration.

This module provides comprehensive test execution capabilities including:
- Test orchestration (unit → integration → E2E)
- Framework detection (pytest, unittest, playwright, etc.)
- Flaky test detection with statistical analysis
- Parallel test execution
- Retry handling for flaky tests
- Test history tracking
- Report aggregation
"""

from src.testing.models import (
    TestRunnerStatus,
    TestRunConfig,
    TestExecutionResult,
    TestSuiteExecution,
    TestDiscoveryResult,
)
from src.testing.orchestrator import (
    TestOrchestrator,
    get_test_orchestrator,
    testing_stage_handler,
)
from src.testing.history import (
    TestHistoryTracker,
    get_history_tracker,
)
from src.testing.flaky_detector import (
    FlakyTestDetector,
    get_flaky_detector,
)
from src.testing.report_aggregator import (
    TestReportAggregator,
    get_report_aggregator,
)
from src.testing.parallel_executor import (
    ParallelTestExecutor,
    ParallelExecutionConfig,
    get_parallel_executor,
)
from src.testing.framework_detector import (
    TestFramework,
    TestFrameworkDetector,
    FrameworkDetectionResult,
    ProjectTestConfig,
    get_framework_detector,
)
from src.testing.retry_handler import (
    RetryStrategy,
    RetryConfig,
    RetryResult,
    RetryHandler,
    SelectiveRetryHandler,
    get_retry_handler,
    get_selective_retry_handler,
)
from src.testing.pipeline_integration import (
    TestPipelineIntegration,
    get_test_pipeline_integration,
    setup_test_pipeline,
)


__all__ = [
    # Models
    "TestRunnerStatus",
    "TestRunConfig",
    "TestExecutionResult",
    "TestSuiteExecution",
    "TestDiscoveryResult",
    # Orchestrator
    "TestOrchestrator",
    "get_test_orchestrator",
    "testing_stage_handler",
    # History
    "TestHistoryTracker",
    "get_history_tracker",
    # Flaky detection
    "FlakyTestDetector",
    "get_flaky_detector",
    # Report aggregation
    "TestReportAggregator",
    "get_report_aggregator",
    # Parallel execution
    "ParallelTestExecutor",
    "ParallelExecutionConfig",
    "get_parallel_executor",
    # Framework detection
    "TestFramework",
    "TestFrameworkDetector",
    "FrameworkDetectionResult",
    "ProjectTestConfig",
    "get_framework_detector",
    # Retry handling
    "RetryStrategy",
    "RetryConfig",
    "RetryResult",
    "RetryHandler",
    "SelectiveRetryHandler",
    "get_retry_handler",
    "get_selective_retry_handler",
    # Pipeline integration
    "TestPipelineIntegration",
    "get_test_pipeline_integration",
    "setup_test_pipeline",
]
