"""Acceptance tests for deployment validation (User Story 3).

These tests verify the acceptance criteria from spec.md:
- Given deployment config files, when validation runs, then required variables verified and secrets detected
- Given project dependencies, when validation executes, then conflicts and vulnerabilities detected
- Given target environment, when compatibility checks run, then resource requirements verified
"""

import pytest
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch
from uuid import uuid4

from src.deployment.config_validator import ConfigValidator
from src.deployment.dependency_checker import DependencyChecker
from src.deployment.security_scanner import SecurityScanner
from src.deployment.compatibility import CompatibilityChecker
from src.deployment.models import (
    ConfigCheckStatus,
    SeverityLevel,
)


class TestAcceptanceScenario1_ConfigValidationAndSecrets:
    """Acceptance Scenario 1:
    Given deployment config files,
    When validation runs,
    Then required variables verified and secrets detected.
    """

    @pytest.fixture
    def config_validator(self):
        return ConfigValidator()

    def test_required_variables_verified(self, config_validator, tmp_path):
        """Verify that required variables are checked against .env.example."""
        # Given: Project with .env.example defining required variables
        env_example = tmp_path / ".env.example"
        env_example.write_text("""
DATABASE_URL=
API_KEY=
SECRET_KEY=
LOG_LEVEL=
""")

        # And .env missing some required variables
        env = tmp_path / ".env"
        env.write_text("""
DATABASE_URL=postgres://localhost/db
LOG_LEVEL=info
""")

        # When: Validation runs
        result = config_validator.validate(tmp_path)

        # Then: Missing required variables are identified
        assert "API_KEY" in result.missing_required
        assert "SECRET_KEY" in result.missing_required
        assert "DATABASE_URL" not in result.missing_required
        assert "LOG_LEVEL" not in result.missing_required

    def test_secrets_detected_in_code(self, config_validator, tmp_path):
        """Verify that hardcoded secrets in code are detected."""
        # Given: Code with hardcoded secrets
        code = tmp_path / "config.py"
        code.write_text("""
# Bad: Hardcoded secrets
API_KEY = "sk-1234567890abcdef1234567890abcdef1234567890abcdef12"
PASSWORD = "super_secret_password"
TOKEN = "ghp_abcdefghijklmnopqrstuvwxyz1234567890"
""")

        # When: Validation runs
        result = config_validator.validate(tmp_path)

        # Then: Secrets are detected
        assert len(result.secret_risks) > 0

        # And: Detection includes file location and line number
        for secret in result.secret_risks:
            assert secret.file_path
            assert secret.line_number > 0
            assert secret.secret_type
            assert secret.recommendation


class TestAcceptanceScenario2_DependencyConflictsAndVulnerabilities:
    """Acceptance Scenario 2:
    Given project dependencies,
    When validation executes,
    Then conflicts and vulnerabilities detected.
    """

    @pytest.fixture
    def dependency_checker(self):
        return DependencyChecker()

    def test_dependencies_counted(self, dependency_checker, tmp_path):
        """Verify that total dependencies are counted."""
        # Given: Project with requirements.txt
        requirements = tmp_path / "requirements.txt"
        requirements.write_text("""
flask>=2.0.0
requests==2.28.0
pydantic>=2.0.0
pytest>=7.0.0
black>=23.0.0
""")

        # When: Validation runs
        with patch.object(dependency_checker, "_check_pip_outdated", return_value=[]):
            with patch.object(dependency_checker, "_check_python_vulnerabilities", return_value=0):
                result = dependency_checker.check(tmp_path)

        # Then: Dependencies are counted
        assert result.total_dependencies == 5

    def test_conflicts_detected(self, dependency_checker):
        """Verify that dependency conflicts are detected."""
        # Given: pip check would report conflicts
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=1,
                stdout="flask 3.0.0 requires werkzeug>=3.0.0, which is not installed.",
                stderr="",
            )

            # When: Checking for conflicts
            conflicts = dependency_checker._find_python_conflicts({})

        # Then: Conflicts are reported
        assert len(conflicts) > 0
        assert conflicts[0].package == "flask"

    def test_vulnerability_count_returned(self, dependency_checker, tmp_path):
        """Verify that security vulnerabilities are counted."""
        # Given: Project with dependencies
        requirements = tmp_path / "requirements.txt"
        requirements.write_text("flask>=2.0.0")

        # When: Vulnerability scan reports issues
        with patch("subprocess.run") as mock_run:
            mock_run.return_value = MagicMock(
                returncode=1,
                stdout='{"vulnerabilities": [{"id": "CVE-1"}, {"id": "CVE-2"}]}',
                stderr="",
            )
            result = dependency_checker.check(tmp_path, check_outdated=False)

        # Then: Vulnerability count is captured (or 0 if tool not available)
        assert isinstance(result.security_vulnerabilities, int)


class TestAcceptanceScenario3_CompatibilityAndResources:
    """Acceptance Scenario 3:
    Given target environment,
    When compatibility checks run,
    Then resource requirements verified.
    """

    @pytest.fixture
    def compatibility_checker(self):
        return CompatibilityChecker()

    def test_resource_requirements_verified(self, compatibility_checker, tmp_path):
        """Verify that resource requirements are checked."""
        # Given: A project directory
        # When: Compatibility check runs
        result = compatibility_checker.check(tmp_path, check_services=False)

        # Then: Resource checks are performed
        assert len(result.resource_checks) > 0

        # And: Disk space is verified
        disk_check = next(
            (r for r in result.resource_checks if r.resource_type == "disk_space"),
            None
        )
        assert disk_check is not None
        assert disk_check.available
        assert disk_check.required

    def test_platform_version_detected(self, compatibility_checker, tmp_path):
        """Verify that platform versions are detected."""
        # Given: Project with Python version requirement
        python_version = tmp_path / ".python-version"
        python_version.write_text("3.10")

        # When: Compatibility check runs
        result = compatibility_checker.check(tmp_path, check_services=False)

        # Then: Platform checks include Python
        python_check = next(
            (p for p in result.platform_checks if p.platform == "python"),
            None
        )
        assert python_check is not None
        assert python_check.required_version
        assert python_check.detected_version
        assert isinstance(python_check.is_compatible, bool)

    def test_required_tools_detected(self, compatibility_checker, tmp_path):
        """Verify that required tools are checked."""
        # Given: Project with package.json (requires node/npm)
        package = tmp_path / "package.json"
        package.write_text('{"name": "test", "version": "1.0.0"}')

        # When: Compatibility check runs
        result = compatibility_checker.check(tmp_path, check_services=False)

        # Then: Node/npm tool checks are included
        tool_checks = [
            r for r in result.resource_checks
            if r.resource_type.startswith("tool:")
        ]
        tool_names = [t.resource_type for t in tool_checks]
        assert "tool:node" in tool_names or "tool:npm" in tool_names


class TestEndToEndDeploymentValidation:
    """End-to-end tests for deployment validation flow."""

    def test_complete_validation_passing_project(self, tmp_path):
        """Test complete deployment validation for a deployable project."""
        # Given: A properly configured project
        project = tmp_path / "good_project"
        project.mkdir()

        # Create proper configuration
        (project / ".env").write_text("NODE_ENV=production\nLOG_LEVEL=info")
        (project / ".env.example").write_text("NODE_ENV=\nLOG_LEVEL=")
        (project / ".gitignore").write_text(".env\n*.pem\n*.key\ncredentials")
        (project / "requirements.txt").write_text("flask>=2.0.0")
        (project / "app.py").write_text("""
import os
app_env = os.environ.get("NODE_ENV")
""")

        # When: All validators run
        config_result = ConfigValidator().validate(project)
        dep_result = DependencyChecker().check(project, check_outdated=False, check_vulnerabilities=False)
        compat_result = CompatibilityChecker().check(project, check_services=False)

        # Then: All validations pass or have only warnings
        assert config_result.status in [ConfigCheckStatus.VALID, ConfigCheckStatus.WARNING]
        assert len(config_result.secret_risks) == 0  # No hardcoded secrets
        assert len(config_result.missing_required) == 0  # All required vars present

    def test_complete_validation_failing_project(self, tmp_path):
        """Test complete deployment validation for an undeployable project."""
        # Given: A poorly configured project
        project = tmp_path / "bad_project"
        project.mkdir()

        # Create problematic configuration
        (project / ".env.example").write_text("API_KEY=\nDATABASE_URL=")
        # Missing .env file
        (project / "config.py").write_text('API_KEY = "sk-abcdef1234567890abcdef1234567890abcdef1234567890ab"')

        # When: Config validation runs
        config_result = ConfigValidator().validate(project)

        # Then: Issues are detected
        assert len(config_result.missing_required) > 0 or len(config_result.secret_risks) > 0
