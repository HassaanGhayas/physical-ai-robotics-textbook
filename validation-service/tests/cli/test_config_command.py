"""Tests for CLI config command."""

import pytest
from unittest.mock import MagicMock, patch

from typer.testing import CliRunner
import typer

from src.cli.commands.config import show_config, validate_config


@pytest.fixture
def runner():
    """Create CLI test runner."""
    return CliRunner()


@pytest.fixture
def mock_settings():
    """Mock settings for testing."""
    with patch("src.cli.commands.config.get_settings") as mock:
        settings = MagicMock()
        settings.environment = "development"
        settings.log_level = "INFO"
        settings.enable_quality_validation = True
        settings.enable_test_execution = True
        settings.enable_deployment_validation = True
        settings.pipeline_timeout_minutes = 15
        settings.quality = MagicMock()
        settings.quality.min_coverage_percentage = 80.0
        settings.quality.max_cyclomatic_complexity = 10
        settings.testing = MagicMock()
        settings.testing.parallel_execution = True
        settings.testing.flaky_detection_enabled = True
        mock.return_value = settings
        yield settings


# Create typer app for testing
config_app = typer.Typer()
config_app.command(name="show")(show_config)
config_app.command(name="validate")(validate_config)


class TestConfigShowCommand:
    """Tests for config show command."""

    def test_show_config_success(self, runner, mock_settings):
        """Test showing configuration."""
        result = runner.invoke(config_app, ["show"])

        assert result.exit_code == 0
        assert "development" in result.output
        assert "INFO" in result.output

    def test_show_config_quality_settings(self, runner, mock_settings):
        """Test quality settings display."""
        result = runner.invoke(config_app, ["show"])

        assert "80" in result.output  # Coverage percentage
        assert "10" in result.output  # Max complexity

    def test_show_config_testing_settings(self, runner, mock_settings):
        """Test testing settings display."""
        result = runner.invoke(config_app, ["show"])

        assert "True" in result.output  # Parallel execution


class TestConfigValidateCommand:
    """Tests for config validate command."""

    def test_validate_config_success(self, runner, mock_settings):
        """Test successful config validation."""
        result = runner.invoke(config_app, ["validate"])

        assert result.exit_code == 0
        assert "valid" in result.output.lower()

    def test_validate_config_with_warnings(self, runner, mock_settings):
        """Test config validation with warnings."""
        mock_settings.environment = "development"
        mock_settings.quality.min_coverage_percentage = 40.0

        result = runner.invoke(config_app, ["validate"])

        assert result.exit_code == 0
        assert "Warning" in result.output

    def test_validate_config_error(self, runner):
        """Test config validation error."""
        with patch("src.cli.commands.config.get_settings") as mock:
            mock.side_effect = Exception("Invalid configuration")

            result = runner.invoke(config_app, ["validate"])

            assert result.exit_code == 1
            assert "error" in result.output.lower()


class TestConfigProduction:
    """Tests for production config."""

    def test_show_production_config(self, runner):
        """Test showing production config."""
        with patch("src.cli.commands.config.get_settings") as mock:
            settings = MagicMock()
            settings.environment = "production"
            settings.log_level = "WARNING"
            settings.enable_quality_validation = True
            settings.enable_test_execution = True
            settings.enable_deployment_validation = True
            settings.pipeline_timeout_minutes = 30
            settings.quality = MagicMock()
            settings.quality.min_coverage_percentage = 90.0
            settings.quality.max_cyclomatic_complexity = 8
            settings.testing = MagicMock()
            settings.testing.parallel_execution = True
            settings.testing.flaky_detection_enabled = True
            mock.return_value = settings

            result = runner.invoke(config_app, ["show"])

            assert "production" in result.output
            assert "90" in result.output

    def test_validate_production_config(self, runner):
        """Test validating production config."""
        with patch("src.cli.commands.config.get_settings") as mock:
            settings = MagicMock()
            settings.environment = "production"
            settings.quality = MagicMock()
            settings.quality.min_coverage_percentage = 90.0
            mock.return_value = settings

            result = runner.invoke(config_app, ["validate"])

            assert result.exit_code == 0
            # No development warning in production
