"""Logging configuration for the validation service.

This module provides centralized logging setup with structured output,
log levels, and handlers for console and file output.
"""

import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

from rich.console import Console
from rich.logging import RichHandler
from rich.theme import Theme

from src.core.config import get_settings


# Custom theme for validation status colors
VALIDATION_THEME = Theme(
    {
        "info": "cyan",
        "warning": "yellow",
        "error": "red bold",
        "success": "green bold",
        "stage": "blue bold",
        "progress": "magenta",
    }
)


class ValidationFormatter(logging.Formatter):
    """Custom formatter for validation service logs."""

    def format(self, record: logging.LogRecord) -> str:
        """Format log record with validation context."""
        # Add timestamp
        record.timestamp = datetime.utcnow().isoformat()

        # Add validation context if present
        if not hasattr(record, "run_id"):
            record.run_id = ""
        if not hasattr(record, "stage"):
            record.stage = ""

        return super().format(record)


def setup_logging(
    level: Optional[str] = None,
    log_file: Optional[Path] = None,
    json_format: bool = False,
) -> logging.Logger:
    """Configure logging for the validation service.

    Args:
        level: Log level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_file: Optional file path for log output
        json_format: Whether to use JSON format for logs

    Returns:
        Configured root logger
    """
    settings = get_settings()
    log_level = getattr(logging, (level or settings.log_level).upper())

    # Get root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(log_level)

    # Clear existing handlers
    root_logger.handlers.clear()

    # Console handler with Rich formatting
    console = Console(theme=VALIDATION_THEME)
    console_handler = RichHandler(
        console=console,
        show_time=True,
        show_path=False,
        rich_tracebacks=True,
        tracebacks_show_locals=settings.environment == "development",
    )
    console_handler.setLevel(log_level)
    console_handler.setFormatter(ValidationFormatter())
    root_logger.addHandler(console_handler)

    # File handler if path provided
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setLevel(log_level)

        if json_format:
            file_handler.setFormatter(
                logging.Formatter(
                    '{"timestamp": "%(timestamp)s", "level": "%(levelname)s", '
                    '"logger": "%(name)s", "message": "%(message)s", '
                    '"run_id": "%(run_id)s", "stage": "%(stage)s"}'
                )
            )
        else:
            file_handler.setFormatter(
                logging.Formatter(
                    "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
                )
            )

        root_logger.addHandler(file_handler)

    # Suppress noisy loggers
    logging.getLogger("httpx").setLevel(logging.WARNING)
    logging.getLogger("httpcore").setLevel(logging.WARNING)
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)

    return root_logger


def get_logger(name: str) -> logging.Logger:
    """Get a logger for a specific module.

    Args:
        name: Module name (typically __name__)

    Returns:
        Logger instance
    """
    return logging.getLogger(name)


class ValidationLoggerAdapter(logging.LoggerAdapter):
    """Logger adapter that adds validation context to log records."""

    def __init__(
        self,
        logger: logging.Logger,
        run_id: Optional[str] = None,
        stage: Optional[str] = None,
    ):
        """Initialize adapter with validation context.

        Args:
            logger: Base logger
            run_id: Current validation run ID
            stage: Current pipeline stage
        """
        super().__init__(logger, {"run_id": run_id or "", "stage": stage or ""})

    def process(self, msg, kwargs):
        """Add validation context to log record."""
        kwargs.setdefault("extra", {}).update(self.extra)
        return msg, kwargs

    def with_stage(self, stage: str) -> "ValidationLoggerAdapter":
        """Create new adapter with updated stage.

        Args:
            stage: New stage name

        Returns:
            New adapter with updated stage
        """
        return ValidationLoggerAdapter(
            self.logger,
            run_id=self.extra.get("run_id"),
            stage=stage,
        )


def log_validation_start(
    logger: logging.Logger,
    repository: str,
    branch: str,
    commit_sha: str,
) -> None:
    """Log validation pipeline start.

    Args:
        logger: Logger to use
        repository: Repository name
        branch: Branch name
        commit_sha: Commit SHA
    """
    logger.info(
        f"Starting validation pipeline for {repository}@{branch} (commit: {commit_sha[:8]})"
    )


def log_stage_start(logger: logging.Logger, stage: str) -> None:
    """Log pipeline stage start.

    Args:
        logger: Logger to use
        stage: Stage name
    """
    logger.info(f"[stage]{stage}[/stage] Starting stage", extra={"stage": stage})


def log_stage_complete(
    logger: logging.Logger,
    stage: str,
    status: str,
    duration: float,
) -> None:
    """Log pipeline stage completion.

    Args:
        logger: Logger to use
        stage: Stage name
        status: Stage status (passed/failed)
        duration: Duration in seconds
    """
    status_color = "success" if status == "passed" else "error"
    logger.info(
        f"[stage]{stage}[/stage] [{status_color}]{status.upper()}[/{status_color}] "
        f"in {duration:.2f}s",
        extra={"stage": stage},
    )


def log_validation_complete(
    logger: logging.Logger,
    status: str,
    duration: float,
) -> None:
    """Log validation pipeline completion.

    Args:
        logger: Logger to use
        status: Final status
        duration: Total duration in seconds
    """
    status_color = "success" if status == "passed" else "error"
    logger.info(
        f"Validation [{status_color}]{status.upper()}[/{status_color}] "
        f"in {duration:.2f}s"
    )
