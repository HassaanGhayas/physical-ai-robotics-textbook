"""Unit tests for dependency checker service.

Tests the DependencyChecker class for dependency validation.
"""

import pytest
from pathlib import Path
from unittest.mock import patch, MagicMock

from src.deployment.dependency_checker import DependencyChecker, get_dependency_checker
from src.deployment.models import ConfigCheckStatus


class TestDependencyChecker:
    """Tests for DependencyChecker class."""

    @pytest.fixture
    def checker(self):
        """Create dependency checker instance."""
        return DependencyChecker()

    @pytest.fixture
    def python_project(self, tmp_path):
        """Create Python project with requirements.txt."""
        req_file = tmp_path / "requirements.txt"
        req_file.write_text("""
flask>=2.0.0
requests==2.28.0
pydantic>=1.10,<2.0
pytest>=7.0.0
""")
        return tmp_path

    @pytest.fixture
    def npm_project(self, tmp_path):
        """Create npm project with package.json."""
        pkg_file = tmp_path / "package.json"
        pkg_file.write_text("""{
    "name": "test-project",
    "version": "1.0.0",
    "dependencies": {
        "express": "^4.18.0",
        "lodash": "^4.17.21"
    },
    "devDependencies": {
        "jest": "^29.0.0"
    }
}""")
        return tmp_path

    def test_detect_pip_manager(self, checker, python_project):
        """Test detection of pip package manager."""
        manager = checker._detect_package_manager(python_project)
        assert manager == "pip"

    def test_detect_npm_manager(self, checker, npm_project):
        """Test detection of npm package manager."""
        manager = checker._detect_package_manager(npm_project)
        assert manager == "npm"

    def test_detect_no_manager(self, checker, tmp_path):
        """Test detection when no package manager files exist."""
        manager = checker._detect_package_manager(tmp_path)
        assert manager is None

    def test_parse_requirements(self, checker, python_project):
        """Test parsing of requirements.txt."""
        requirements = checker._parse_requirements(python_project)

        assert "flask" in requirements
        assert "requests" in requirements
        assert "pydantic" in requirements

    def test_check_returns_result(self, checker, python_project):
        """Test that check returns a DependencyValidationResult."""
        with patch.object(checker, "_check_pip_outdated", return_value=[]):
            with patch.object(checker, "_check_python_vulnerabilities", return_value=0):
                result = checker.check(python_project)

        assert result.total_dependencies > 0
        assert result.status in [ConfigCheckStatus.VALID, ConfigCheckStatus.WARNING]

    def test_check_npm_project(self, checker, npm_project):
        """Test checking npm project dependencies."""
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=0,
                stdout="{}",
                stderr="",
            )
            result = checker.check(npm_project)

        assert result.total_dependencies >= 0

    def test_check_handles_empty_project(self, checker, tmp_path):
        """Test checking project with no dependencies."""
        result = checker.check(tmp_path)

        assert result.total_dependencies == 0
        assert "No recognized package manager found" in result.warnings


class TestDependencyConflictDetection:
    """Tests for dependency conflict detection."""

    @pytest.fixture
    def checker(self):
        return DependencyChecker()

    def test_find_python_conflicts_with_pip_check(self, checker, tmp_path):
        """Test finding Python dependency conflicts."""
        # Create minimal requirements
        req = tmp_path / "requirements.txt"
        req.write_text("flask>=2.0.0")

        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=0,
                stdout="",
                stderr="",
            )
            conflicts = checker._find_python_conflicts({"flask": ">=2.0.0"})

        # No conflicts when pip check passes
        assert len(conflicts) == 0

    def test_conflict_detected_when_pip_check_fails(self, checker):
        """Test that conflicts are detected when pip check fails."""
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=1,
                stdout="package-a requires package-b which is not installed",
                stderr="",
            )
            conflicts = checker._find_python_conflicts({})

        assert len(conflicts) > 0


class TestGetDependencyChecker:
    """Tests for get_dependency_checker factory function."""

    def test_returns_checker_instance(self):
        """Test that get_dependency_checker returns a DependencyChecker."""
        checker = get_dependency_checker()
        assert isinstance(checker, DependencyChecker)
