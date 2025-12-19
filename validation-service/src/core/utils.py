"""Utility functions for the validation service.

This module provides common utilities for UUID generation, datetime handling,
file operations, and other cross-cutting concerns.
"""

import hashlib
import os
import subprocess
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional
from uuid import UUID, uuid4


def generate_uuid() -> UUID:
    """Generate a new UUID.

    Returns:
        New UUID4 identifier
    """
    return uuid4()


def generate_uuid_str() -> str:
    """Generate a new UUID as a string.

    Returns:
        New UUID4 as string
    """
    return str(uuid4())


def now_utc() -> datetime:
    """Get current UTC timestamp.

    Returns:
        Current datetime in UTC
    """
    return datetime.now(timezone.utc)


def parse_iso_datetime(value: str) -> datetime:
    """Parse an ISO 8601 datetime string.

    Args:
        value: ISO 8601 formatted string

    Returns:
        Parsed datetime object
    """
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def format_iso_datetime(dt: datetime) -> str:
    """Format datetime as ISO 8601 string.

    Args:
        dt: Datetime to format

    Returns:
        ISO 8601 formatted string
    """
    return dt.isoformat()


def format_duration(seconds: float) -> str:
    """Format duration in human-readable format.

    Args:
        seconds: Duration in seconds

    Returns:
        Human-readable duration string (e.g., "2m 30s", "1h 15m")
    """
    if seconds < 60:
        return f"{seconds:.1f}s"
    elif seconds < 3600:
        minutes = int(seconds // 60)
        secs = seconds % 60
        return f"{minutes}m {secs:.0f}s"
    else:
        hours = int(seconds // 3600)
        minutes = int((seconds % 3600) // 60)
        return f"{hours}h {minutes}m"


def hash_string(value: str) -> str:
    """Generate a SHA256 hash of a string.

    Args:
        value: String to hash

    Returns:
        Hex-encoded SHA256 hash
    """
    return hashlib.sha256(value.encode()).hexdigest()


def hash_file(path: Path) -> str:
    """Generate a SHA256 hash of a file.

    Args:
        path: Path to file

    Returns:
        Hex-encoded SHA256 hash
    """
    sha256 = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(8192), b""):
            sha256.update(chunk)
    return sha256.hexdigest()


def get_git_info(repo_path: Path) -> dict:
    """Get Git repository information.

    Args:
        repo_path: Path to Git repository

    Returns:
        Dictionary with repository, branch, and commit info
    """
    result = {
        "repository": "",
        "branch": "",
        "commit_sha": "",
        "is_git_repo": False,
    }

    git_dir = repo_path / ".git"
    if not git_dir.exists():
        return result

    result["is_git_repo"] = True

    try:
        # Get repository name from remote
        remote_result = subprocess.run(
            ["git", "remote", "get-url", "origin"],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )
        if remote_result.returncode == 0:
            remote_url = remote_result.stdout.strip()
            # Extract repo name from URL
            if remote_url.endswith(".git"):
                remote_url = remote_url[:-4]
            result["repository"] = remote_url.split("/")[-1]

        # Get current branch
        branch_result = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )
        if branch_result.returncode == 0:
            result["branch"] = branch_result.stdout.strip()

        # Get current commit SHA
        commit_result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=repo_path,
            capture_output=True,
            text=True,
        )
        if commit_result.returncode == 0:
            result["commit_sha"] = commit_result.stdout.strip()

    except (subprocess.SubprocessError, OSError):
        pass

    return result


def find_project_root(start_path: Path) -> Optional[Path]:
    """Find the project root directory.

    Searches upward from start_path for common project markers:
    - .git directory
    - pyproject.toml
    - setup.py
    - package.json

    Args:
        start_path: Path to start searching from

    Returns:
        Project root path or None if not found
    """
    markers = [".git", "pyproject.toml", "setup.py", "package.json"]
    current = start_path.resolve()

    while current != current.parent:
        for marker in markers:
            if (current / marker).exists():
                return current
        current = current.parent

    return None


def relative_path(path: Path, base: Path) -> str:
    """Get path relative to base directory.

    Args:
        path: Path to convert
        base: Base directory

    Returns:
        Relative path as string, or absolute path if not under base
    """
    try:
        return str(path.resolve().relative_to(base.resolve()))
    except ValueError:
        return str(path)


def ensure_directory(path: Path) -> Path:
    """Ensure a directory exists, creating it if necessary.

    Args:
        path: Directory path

    Returns:
        The same path (for chaining)
    """
    path.mkdir(parents=True, exist_ok=True)
    return path


def safe_filename(name: str, max_length: int = 255) -> str:
    """Convert a string to a safe filename.

    Removes or replaces characters that are invalid in filenames.

    Args:
        name: Original name
        max_length: Maximum filename length

    Returns:
        Safe filename string
    """
    # Replace invalid characters
    invalid_chars = '<>:"/\\|?*'
    for char in invalid_chars:
        name = name.replace(char, "_")

    # Replace whitespace
    name = name.replace(" ", "_")

    # Remove leading/trailing dots and spaces
    name = name.strip(". ")

    # Truncate if too long
    if len(name) > max_length:
        name = name[:max_length]

    return name or "unnamed"


def count_lines(path: Path) -> int:
    """Count lines in a file.

    Args:
        path: Path to file

    Returns:
        Number of lines
    """
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return sum(1 for _ in f)
    except OSError:
        return 0


def get_file_extension(path: Path) -> str:
    """Get lowercase file extension without the dot.

    Args:
        path: File path

    Returns:
        Extension without dot (e.g., "py", "ts")
    """
    return path.suffix.lstrip(".").lower()


def is_python_file(path: Path) -> bool:
    """Check if path is a Python file.

    Args:
        path: File path

    Returns:
        True if Python file
    """
    return get_file_extension(path) == "py"


def is_test_file(path: Path) -> bool:
    """Check if path is a test file.

    Args:
        path: File path

    Returns:
        True if test file
    """
    name = path.stem.lower()
    return name.startswith("test_") or name.endswith("_test")


def get_environment_variable(name: str, default: Optional[str] = None) -> Optional[str]:
    """Get environment variable value.

    Args:
        name: Variable name
        default: Default value if not set

    Returns:
        Variable value or default
    """
    return os.environ.get(name, default)


def set_environment_variable(name: str, value: str) -> None:
    """Set environment variable.

    Args:
        name: Variable name
        value: Variable value
    """
    os.environ[name] = value
