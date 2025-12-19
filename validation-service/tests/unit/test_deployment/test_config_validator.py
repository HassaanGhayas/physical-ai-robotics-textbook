"""Unit tests for configuration validator service.

Tests the ConfigValidator class for environment and configuration validation.
"""

import pytest
from pathlib import Path

from src.deployment.config_validator import ConfigValidator, get_config_validator
from src.deployment.models import ConfigCheckStatus


class TestConfigValidator:
    """Tests for ConfigValidator class."""

    @pytest.fixture
    def validator(self):
        """Create config validator instance."""
        return ConfigValidator()

    @pytest.fixture
    def project_with_valid_env(self, tmp_path):
        """Create project with valid .env file."""
        env_file = tmp_path / ".env"
        env_file.write_text("""
NODE_ENV=production
LOG_LEVEL=info
API_KEY="test-api-key-12345"
DATABASE_URL=postgres://localhost/test
""")
        return tmp_path

    @pytest.fixture
    def project_with_invalid_env(self, tmp_path):
        """Create project with invalid .env file."""
        env_file = tmp_path / ".env"
        env_file.write_text("""
NODE_ENV=production
INVALID_LINE_WITHOUT_EQUALS
LOG_LEVEL=
""")
        return tmp_path

    @pytest.fixture
    def project_with_secrets(self, tmp_path):
        """Create project with hardcoded secrets."""
        code_file = tmp_path / "config.py"
        code_file.write_text("""
API_KEY = "sk-1234567890abcdef1234567890abcdef1234567890abcdef12"
PASSWORD = "super_secret_password"
DATABASE_URL = "postgres://user:password@localhost/db"
""")
        return tmp_path

    def test_validate_valid_env(self, validator, project_with_valid_env):
        """Test validation of valid environment file."""
        result = validator.validate(project_with_valid_env)

        assert result.status in [ConfigCheckStatus.VALID, ConfigCheckStatus.WARNING]
        assert len(result.checks) > 0

    def test_validate_invalid_env(self, validator, project_with_invalid_env):
        """Test validation of invalid environment file."""
        result = validator.validate(project_with_invalid_env)

        # Should detect the invalid line
        invalid_checks = [
            c for c in result.checks if c.status == ConfigCheckStatus.INVALID
        ]
        assert len(invalid_checks) > 0 or len(result.missing_required) > 0

    def test_detect_hardcoded_secrets(self, validator, project_with_secrets):
        """Test detection of hardcoded secrets in code."""
        result = validator.validate(project_with_secrets)

        # Should detect at least one secret
        assert len(result.secret_risks) > 0

    def test_check_env_completeness(self, validator, tmp_path):
        """Test checking .env against .env.example."""
        # Create .env.example with required vars
        example = tmp_path / ".env.example"
        example.write_text("""
REQUIRED_VAR=
ANOTHER_REQUIRED=
OPTIONAL_VAR=
""")

        # Create .env missing some vars
        env = tmp_path / ".env"
        env.write_text("""
REQUIRED_VAR=value
""")

        result = validator.validate(tmp_path)

        assert "ANOTHER_REQUIRED" in result.missing_required

    def test_sensitive_values_redacted(self, validator, tmp_path):
        """Test that sensitive values are redacted in output."""
        env = tmp_path / ".env"
        env.write_text("""
PASSWORD=my_secret_password
API_KEY=abcdef12345
""")

        result = validator.validate(tmp_path)

        # Find password check
        password_check = next(
            (c for c in result.checks if c.name == "PASSWORD"), None
        )
        if password_check:
            assert password_check.is_sensitive
            assert password_check.actual_value == "***"


class TestConfigValidatorPatterns:
    """Tests for secret detection patterns."""

    @pytest.fixture
    def validator(self):
        return ConfigValidator()

    def test_detect_openai_key(self, validator, tmp_path):
        """Test detection of OpenAI API key pattern."""
        code = tmp_path / "test.py"
        code.write_text('OPENAI_KEY = "sk-abcdef1234567890abcdef1234567890abcdef1234567890ab"')

        result = validator.validate(tmp_path)
        assert any("OpenAI" in s.secret_type or "API Key" in s.secret_type
                   for s in result.secret_risks)

    def test_detect_github_token(self, validator, tmp_path):
        """Test detection of GitHub PAT pattern."""
        code = tmp_path / "test.py"
        code.write_text('TOKEN = "ghp_abcdefghijklmnopqrstuvwxyz1234567890"')

        result = validator.validate(tmp_path)
        assert any("GitHub" in s.secret_type for s in result.secret_risks)


class TestGetConfigValidator:
    """Tests for get_config_validator factory function."""

    def test_returns_validator_instance(self):
        """Test that get_config_validator returns a ConfigValidator."""
        validator = get_config_validator()
        assert isinstance(validator, ConfigValidator)
