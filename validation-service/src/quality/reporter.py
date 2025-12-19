"""Quality reporter service.

This module aggregates results from all quality tools (linter, complexity,
coverage, security) into a unified CodeQualityReport.
"""

import logging
from datetime import datetime
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.quality.complexity import ComplexityAnalyzer, get_complexity_analyzer
from src.quality.coverage import CoverageCalculator, get_coverage_calculator
from src.quality.linter import LinterService, get_linter_service
from src.quality.models import (
    CheckStatus,
    CodeQualityReport,
    ComplexityResults,
    CoverageResults,
    LintingResults,
    QualitySummary,
    SecurityResults,
)
from src.quality.security import SecurityScanner, get_security_scanner


logger = logging.getLogger(__name__)


class QualityReporter:
    """Service for generating quality reports."""

    def __init__(
        self,
        linter: Optional[LinterService] = None,
        complexity_analyzer: Optional[ComplexityAnalyzer] = None,
        coverage_calculator: Optional[CoverageCalculator] = None,
        security_scanner: Optional[SecurityScanner] = None,
    ):
        """Initialize the quality reporter.

        Args:
            linter: Linter service (optional, uses default if not provided)
            complexity_analyzer: Complexity analyzer (optional)
            coverage_calculator: Coverage calculator (optional)
            security_scanner: Security scanner (optional)
        """
        self.settings = get_settings()
        self.linter = linter or get_linter_service()
        self.complexity = complexity_analyzer or get_complexity_analyzer()
        self.coverage = coverage_calculator or get_coverage_calculator()
        self.security = security_scanner or get_security_scanner()

    def generate_report(
        self,
        project_path: Path,
        run_id: UUID,
        skip_coverage: bool = False,
    ) -> CodeQualityReport:
        """Generate a comprehensive quality report.

        Args:
            project_path: Path to project to analyze
            run_id: ID of the validation run
            skip_coverage: Skip coverage calculation (faster but incomplete)

        Returns:
            CodeQualityReport with all metrics
        """
        logger.info(f"Generating quality report for {project_path}")

        # Collect results from all tools
        linting_results = self._run_linting(project_path)
        complexity_results = self._run_complexity(project_path)
        coverage_results = (
            CoverageResults() if skip_coverage else self._run_coverage(project_path)
        )
        security_results = self._run_security(project_path)
        maintainability = self._get_maintainability(project_path)

        # Determine overall status
        status = self._determine_status(
            linting_results,
            complexity_results,
            coverage_results,
            security_results,
        )

        # Check if quality gate passed
        quality_gate_passed = self._check_quality_gate(
            linting_results,
            complexity_results,
            coverage_results,
            security_results,
        )

        return CodeQualityReport(
            run_id=run_id,
            timestamp=datetime.utcnow(),
            status=status,
            linting=linting_results,
            complexity=complexity_results,
            coverage=coverage_results,
            security=security_results,
            summary=QualitySummary(
                maintainability_index=maintainability,
                quality_gate_passed=quality_gate_passed,
            ),
        )

    def _run_linting(self, project_path: Path) -> LintingResults:
        """Run linting analysis.

        Args:
            project_path: Path to project

        Returns:
            LintingResults
        """
        try:
            return self.linter.run_ruff(project_path)
        except Exception as e:
            logger.error(f"Linting failed: {e}")
            return LintingResults()

    def _run_complexity(self, project_path: Path) -> ComplexityResults:
        """Run complexity analysis.

        Args:
            project_path: Path to project

        Returns:
            ComplexityResults
        """
        try:
            return self.complexity.analyze(project_path)
        except Exception as e:
            logger.error(f"Complexity analysis failed: {e}")
            return ComplexityResults()

    def _run_coverage(self, project_path: Path) -> CoverageResults:
        """Run coverage calculation.

        Args:
            project_path: Path to project

        Returns:
            CoverageResults
        """
        try:
            return self.coverage.calculate(project_path)
        except Exception as e:
            logger.error(f"Coverage calculation failed: {e}")
            return CoverageResults()

    def _run_security(self, project_path: Path) -> SecurityResults:
        """Run security scan.

        Args:
            project_path: Path to project

        Returns:
            SecurityResults
        """
        try:
            code_scan = self.security.scan(project_path)
            dep_scan = self.security.scan_dependencies(project_path)

            # Merge dependency vulnerabilities
            code_scan.vulnerabilities.extend(dep_scan)

            return code_scan
        except Exception as e:
            logger.error(f"Security scan failed: {e}")
            return SecurityResults()

    def _get_maintainability(self, project_path: Path) -> float:
        """Get maintainability index.

        Args:
            project_path: Path to project

        Returns:
            Maintainability index (0-100)
        """
        try:
            return self.complexity.get_maintainability_index(project_path)
        except Exception as e:
            logger.error(f"Maintainability calculation failed: {e}")
            return 0.0

    def _determine_status(
        self,
        linting: LintingResults,
        complexity: ComplexityResults,
        coverage: CoverageResults,
        security: SecurityResults,
    ) -> CheckStatus:
        """Determine overall status from individual results.

        Args:
            linting: Linting results
            complexity: Complexity results
            coverage: Coverage results
            security: Security results

        Returns:
            Overall CheckStatus
        """
        # Failed if any critical issues
        if security.critical_count > 0:
            return CheckStatus.FAILED
        if linting.error_count > 0:
            return CheckStatus.FAILED
        if coverage.line_coverage_percent < self.settings.quality.min_coverage_percentage:
            return CheckStatus.FAILED

        # Warning if non-critical issues
        if security.high_count > 0:
            return CheckStatus.WARNING
        if complexity.max_complexity > self.settings.quality.max_cyclomatic_complexity:
            return CheckStatus.WARNING
        if linting.warning_count > 10:  # Threshold for warnings
            return CheckStatus.WARNING

        return CheckStatus.PASSED

    def _check_quality_gate(
        self,
        linting: LintingResults,
        complexity: ComplexityResults,
        coverage: CoverageResults,
        security: SecurityResults,
    ) -> bool:
        """Check if quality gate passes.

        Args:
            linting: Linting results
            complexity: Complexity results
            coverage: Coverage results
            security: Security results

        Returns:
            True if quality gate passes
        """
        settings = self.settings.quality

        # No critical/high security issues
        if security.critical_count > 0 or security.high_count > 0:
            return False

        # No linting errors
        if linting.error_count > 0:
            return False

        # Coverage meets minimum
        if coverage.line_coverage_percent < settings.min_coverage_percentage:
            return False

        # Complexity within limits
        if complexity.max_complexity > settings.max_cyclomatic_complexity * 2:
            return False

        return True


def get_quality_reporter() -> QualityReporter:
    """Get quality reporter instance.

    Returns:
        QualityReporter instance
    """
    return QualityReporter()
