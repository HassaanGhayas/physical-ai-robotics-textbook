"""Tests for the linter service."""

import json
import pytest
from unittest.mock import Mock, patch, MagicMock
from pathlib import Path

from src.core.exceptions import LinterError
from src.quality.linter import LinterService, get_linter_service
from src.quality.models import Severity, Violation


class TestLinterService:
    """Tests for LinterService."""

    def test_init(self):
        """Test linter service initialization."""
        service = LinterService()
        assert service.settings is not None

    @patch("src.quality.linter.subprocess.run")
    def test_run_ruff_success_no_violations(self, mock_run):
        """Test running Ruff with no violations found."""
        mock_run.return_value = Mock(
            stdout="[]",
            stderr="",
            returncode=0,
        )

        service = LinterService()
        result = service.run_ruff(Path("/test/project"))

        assert result.total_violations == 0
        assert result.error_count == 0
        assert result.warning_count == 0
        assert len(result.violations) == 0

    @patch("src.quality.linter.subprocess.run")
    def test_run_ruff_with_violations(self, mock_run):
        """Test running Ruff with violations found."""
        ruff_output = json.dumps([
            {
                "filename": "src/main.py",
                "location": {"row": 10, "column": 5},
                "code": "E501",
                "message": "Line too long",
            },
            {
                "filename": "src/utils.py",
                "location": {"row": 20, "column": 1},
                "code": "W503",
                "message": "Line break before operator",
            },
        ])

        mock_run.return_value = Mock(
            stdout=ruff_output,
            stderr="",
            returncode=1,
        )

        service = LinterService()
        result = service.run_ruff(Path("/test/project"))

        assert result.total_violations == 2
        assert result.error_count == 1  # E501 is an error
        assert result.warning_count == 1  # W503 is a warning
        assert len(result.violations) == 2

        # Check first violation
        assert result.violations[0].file == "src/main.py"
        assert result.violations[0].line == 10
        assert result.violations[0].column == 5
        assert result.violations[0].rule == "E501"
        assert result.violations[0].severity == Severity.ERROR

        # Check second violation
        assert result.violations[1].file == "src/utils.py"
        assert result.violations[1].rule == "W503"
        assert result.violations[1].severity == Severity.WARNING

    @patch("src.quality.linter.subprocess.run")
    def test_run_ruff_invalid_json(self, mock_run):
        """Test handling of invalid JSON output from Ruff."""
        mock_run.return_value = Mock(
            stdout="invalid json",
            stderr="",
            returncode=0,
        )

        service = LinterService()
        result = service.run_ruff(Path("/test/project"))

        # Should return empty results when JSON parsing fails
        assert result.total_violations == 0

    @patch("src.quality.linter.subprocess.run")
    def test_run_ruff_timeout(self, mock_run):
        """Test handling of timeout."""
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired("ruff", 300)

        service = LinterService()
        with pytest.raises(LinterError) as exc_info:
            service.run_ruff(Path("/test/project"))

        assert "timed out" in str(exc_info.value).lower()

    @patch("src.quality.linter.subprocess.run")
    def test_run_ruff_not_installed(self, mock_run):
        """Test handling when Ruff is not installed."""
        mock_run.side_effect = FileNotFoundError()

        service = LinterService()
        with pytest.raises(LinterError) as exc_info:
            service.run_ruff(Path("/test/project"))

        assert "not installed" in str(exc_info.value).lower()

    def test_map_ruff_severity_error_codes(self):
        """Test severity mapping for error codes."""
        service = LinterService()

        # E and F codes should be errors
        assert service._map_ruff_severity("E501") == Severity.ERROR
        assert service._map_ruff_severity("E101") == Severity.ERROR
        assert service._map_ruff_severity("F401") == Severity.ERROR
        assert service._map_ruff_severity("F841") == Severity.ERROR

    def test_map_ruff_severity_warning_codes(self):
        """Test severity mapping for warning codes."""
        service = LinterService()

        # W, C, B codes should be warnings
        assert service._map_ruff_severity("W503") == Severity.WARNING
        assert service._map_ruff_severity("C901") == Severity.WARNING
        assert service._map_ruff_severity("B001") == Severity.WARNING

    def test_map_ruff_severity_info_codes(self):
        """Test severity mapping for info codes."""
        service = LinterService()

        # I and D codes should be info
        assert service._map_ruff_severity("I001") == Severity.INFO
        assert service._map_ruff_severity("D100") == Severity.INFO

    def test_map_ruff_severity_unknown_code(self):
        """Test severity mapping for unknown codes."""
        service = LinterService()

        # Unknown codes default to warning
        assert service._map_ruff_severity("X001") == Severity.WARNING

    @patch("src.quality.linter.subprocess.run")
    def test_check_file_existing(self, mock_run):
        """Test checking a single existing file."""
        ruff_output = json.dumps([
            {
                "filename": "/test/file.py",
                "location": {"row": 5, "column": 1},
                "code": "E501",
                "message": "Line too long",
            },
        ])

        mock_run.return_value = Mock(
            stdout=ruff_output,
            stderr="",
            returncode=0,
        )

        service = LinterService()
        with patch.object(Path, "exists", return_value=True):
            violations = service.check_file(Path("/test/file.py"))

        assert len(violations) == 1
        assert violations[0].file == "/test/file.py"

    def test_check_file_not_existing(self):
        """Test checking a non-existing file."""
        service = LinterService()
        with patch.object(Path, "exists", return_value=False):
            violations = service.check_file(Path("/test/nonexistent.py"))

        assert len(violations) == 0


class TestGetLinterService:
    """Tests for get_linter_service function."""

    def test_get_linter_service(self):
        """Test getting linter service instance."""
        service = get_linter_service()
        assert isinstance(service, LinterService)
