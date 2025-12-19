"""Integration tests for the quality validation pipeline.

These tests verify that the quality validation system works end-to-end,
from receiving a project path to producing a complete quality report.
"""

import pytest
from pathlib import Path
from uuid import uuid4
from unittest.mock import patch, Mock, MagicMock

from src.core.models import (
    CheckStatus,
    CodeQualityReport,
    ValidationRun,
    TriggerSource,
)
from src.quality.validator import QualityValidator, quality_stage_handler
from src.quality.reporter import QualityReporter


class TestQualityValidatorIntegration:
    """Integration tests for QualityValidator."""

    def test_validate_nonexistent_path(self):
        """Test validation fails gracefully for non-existent path."""
        validator = QualityValidator()
        run_id = uuid4()

        result = validator.validate(
            project_path=Path("/nonexistent/path"),
            run_id=run_id,
        )

        assert result.run_id == run_id
        assert result.status == CheckStatus.FAILED

    @patch("src.quality.validator.get_quality_reporter")
    def test_validate_success(self, mock_get_reporter):
        """Test successful validation with mocked reporter."""
        run_id = uuid4()
        mock_report = CodeQualityReport(
            run_id=run_id,
            status=CheckStatus.PASSED,
        )

        mock_reporter = Mock(spec=QualityReporter)
        mock_reporter.generate_report.return_value = mock_report
        mock_get_reporter.return_value = mock_reporter

        validator = QualityValidator(reporter=mock_reporter)

        # Use a path that "exists" for testing
        with patch.object(Path, "exists", return_value=True):
            result = validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        assert result.run_id == run_id
        assert result.status == CheckStatus.PASSED
        mock_reporter.generate_report.assert_called_once()

    @patch("src.quality.validator.get_quality_reporter")
    def test_validate_with_skip_coverage(self, mock_get_reporter):
        """Test validation with skip_coverage flag."""
        run_id = uuid4()
        mock_report = CodeQualityReport(run_id=run_id)

        mock_reporter = Mock(spec=QualityReporter)
        mock_reporter.generate_report.return_value = mock_report
        mock_get_reporter.return_value = mock_reporter

        validator = QualityValidator(reporter=mock_reporter)

        with patch.object(Path, "exists", return_value=True):
            validator.validate(
                project_path=Path("/test/project"),
                run_id=run_id,
                skip_coverage=True,
            )

        # Verify skip_coverage was passed to reporter
        mock_reporter.generate_report.assert_called_once_with(
            project_path=Path("/test/project"),
            run_id=run_id,
            skip_coverage=True,
        )


class TestQualityStageHandler:
    """Integration tests for quality_stage_handler."""

    @pytest.mark.asyncio
    @patch("src.quality.validator.get_quality_validator")
    async def test_quality_stage_handler_success(self, mock_get_validator):
        """Test quality stage handler in pipeline context."""
        from src.core.pipeline import PipelineContext

        run_id = uuid4()
        mock_report = CodeQualityReport(
            run_id=run_id,
            status=CheckStatus.PASSED,
        )

        mock_validator = Mock(spec=QualityValidator)
        mock_validator.validate.return_value = mock_report
        mock_get_validator.return_value = mock_validator

        # Create pipeline context
        run = ValidationRun(
            run_id=run_id,
            trigger_source=TriggerSource.MANUAL,
            repository="test-repo",
            branch="main",
            commit_sha="abc123",
        )

        context = PipelineContext(
            run=run,
            project_path="/test/project",
        )

        result = await quality_stage_handler(context)

        assert result["status"] == CheckStatus.PASSED
        assert result["report"] == mock_report
        assert context.run.quality_report == mock_report


class TestQualityReporterIntegration:
    """Integration tests for QualityReporter."""

    @patch("src.quality.reporter.get_linter_service")
    @patch("src.quality.reporter.get_complexity_analyzer")
    @patch("src.quality.reporter.get_coverage_calculator")
    @patch("src.quality.reporter.get_security_scanner")
    def test_generate_report_aggregation(
        self,
        mock_security,
        mock_coverage,
        mock_complexity,
        mock_linter,
    ):
        """Test that reporter properly aggregates results from all services."""
        from src.quality.models import (
            LintingResults,
            ComplexityResults,
            CoverageResults,
            SecurityResults,
        )

        # Set up mock services
        mock_linter_service = Mock()
        mock_linter_service.run_ruff.return_value = LintingResults(
            total_violations=2,
            error_count=1,
            warning_count=1,
        )
        mock_linter.return_value = mock_linter_service

        mock_complexity_service = Mock()
        mock_complexity_service.analyze.return_value = ComplexityResults(
            average_complexity=5.0,
            max_complexity=10,
        )
        mock_complexity_service.get_maintainability_index.return_value = 85.0
        mock_complexity.return_value = mock_complexity_service

        mock_coverage_service = Mock()
        mock_coverage_service.calculate.return_value = CoverageResults(
            line_coverage_percent=80.0,
            branch_coverage_percent=70.0,
        )
        mock_coverage.return_value = mock_coverage_service

        mock_security_service = Mock()
        mock_security_service.scan.return_value = SecurityResults()
        mock_security.return_value = mock_security_service

        reporter = QualityReporter()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            report = reporter.generate_report(
                project_path=Path("/test/project"),
                run_id=run_id,
            )

        # Verify results are aggregated
        assert report.run_id == run_id
        assert report.linting.total_violations == 2
        assert report.complexity.average_complexity == 5.0
        assert report.coverage.line_coverage_percent == 80.0


class TestQualityPipelineEndToEnd:
    """End-to-end tests for quality pipeline with fixtures."""

    @pytest.fixture
    def fixtures_path(self):
        """Get path to test fixtures."""
        return Path(__file__).parent.parent / "fixtures" / "sample_code"

    @patch("src.quality.linter.subprocess.run")
    @patch("src.quality.complexity.subprocess.run")
    @patch("src.quality.coverage.subprocess.run")
    @patch("src.quality.security.subprocess.run")
    def test_validate_clean_code(
        self,
        mock_security_run,
        mock_coverage_run,
        mock_complexity_run,
        mock_linter_run,
        fixtures_path,
    ):
        """Test validating clean code passes quality checks."""
        import json

        # Mock all subprocess calls to return clean results
        mock_linter_run.return_value = Mock(stdout="[]", returncode=0)
        mock_complexity_run.return_value = Mock(stdout="{}", returncode=0)
        mock_coverage_run.side_effect = [
            Mock(stdout="", returncode=0),  # pytest
            Mock(
                stdout=json.dumps({"totals": {"percent_covered": 90.0}}),
                returncode=0,
            ),  # coverage
        ]
        mock_security_run.return_value = Mock(stdout="[]", returncode=0)

        validator = QualityValidator()
        run_id = uuid4()

        with patch.object(Path, "exists", return_value=True):
            result = validator.validate(
                project_path=fixtures_path,
                run_id=run_id,
            )

        # Clean code should pass or have minimal issues
        assert result.run_id == run_id
        assert result.linting.error_count == 0
