"""Quality validator entry point.

This module provides the main entry point for quality validation,
integrating with the pipeline orchestrator.
"""

import logging
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import CheckStatus, CodeQualityReport
from src.core.pipeline import PipelineContext, PipelineStage
from src.quality.reporter import QualityReporter, get_quality_reporter


logger = logging.getLogger(__name__)


class QualityValidator:
    """Main quality validation service."""

    def __init__(self, reporter: Optional[QualityReporter] = None):
        """Initialize the quality validator.

        Args:
            reporter: Quality reporter (optional, uses default if not provided)
        """
        self.settings = get_settings()
        self.reporter = reporter or get_quality_reporter()

    def validate(
        self,
        project_path: Path,
        run_id: UUID,
        skip_coverage: bool = False,
    ) -> CodeQualityReport:
        """Validate code quality for a project.

        Args:
            project_path: Path to project to validate
            run_id: ID of the validation run
            skip_coverage: Skip coverage calculation for faster validation

        Returns:
            CodeQualityReport with validation results
        """
        logger.info(f"Starting quality validation for {project_path}")

        if not project_path.exists():
            logger.error(f"Project path does not exist: {project_path}")
            return CodeQualityReport(
                run_id=run_id,
                status=CheckStatus.FAILED,
            )

        report = self.reporter.generate_report(
            project_path=project_path,
            run_id=run_id,
            skip_coverage=skip_coverage,
        )

        self._log_summary(report)
        return report

    def _log_summary(self, report: CodeQualityReport) -> None:
        """Log a summary of the quality report.

        Args:
            report: Quality report to summarize
        """
        logger.info(f"Quality validation {report.status.value}")
        logger.info(
            f"  Linting: {report.linting.total_violations} violations "
            f"({report.linting.error_count} errors, {report.linting.warning_count} warnings)"
        )
        logger.info(
            f"  Complexity: avg={report.complexity.average_complexity:.2f}, "
            f"max={report.complexity.max_complexity}, "
            f"high_complexity_functions={len(report.complexity.high_complexity_functions)}"
        )
        logger.info(
            f"  Coverage: {report.coverage.line_coverage_percent:.1f}% lines, "
            f"{report.coverage.branch_coverage_percent:.1f}% branches"
        )
        logger.info(
            f"  Security: {report.security.critical_count} critical, "
            f"{report.security.high_count} high, "
            f"{report.security.medium_count} medium, "
            f"{report.security.low_count} low"
        )
        logger.info(
            f"  Quality Gate: {'PASSED' if report.summary.quality_gate_passed else 'FAILED'}"
        )


async def quality_stage_handler(context: PipelineContext) -> dict:
    """Pipeline stage handler for quality validation.

    This function is registered with the pipeline orchestrator to handle
    the quality validation stage.

    Args:
        context: Pipeline execution context

    Returns:
        Dictionary with status and report
    """
    validator = get_quality_validator()

    report = validator.validate(
        project_path=Path(context.project_path),
        run_id=context.run.run_id,
        skip_coverage=False,
    )

    # Attach report to the run
    context.run.quality_report = report

    return {
        "status": report.status,
        "report": report,
    }


def get_quality_validator() -> QualityValidator:
    """Get quality validator instance.

    Returns:
        QualityValidator instance
    """
    return QualityValidator()


def setup_quality_pipeline() -> None:
    """Register quality validation handlers with the pipeline orchestrator."""
    from src.core.pipeline import get_pipeline

    pipeline = get_pipeline()
    pipeline.register_stage_handler(PipelineStage.QUALITY, quality_stage_handler)
    logger.info("Quality pipeline handlers registered")
