"""JSON storage system for validation results and test history.

This module provides file-based storage for validation data using JSON files.
It implements the storage strategy defined in the data model specification.
"""

import json
import os
from datetime import datetime, timedelta
from pathlib import Path
from typing import Optional
from uuid import UUID

from src.core.config import get_settings
from src.core.models import (
    FlakyTestRecord,
    TestCase,
    ValidationRun,
    ValidationStatus,
)


class JSONEncoder(json.JSONEncoder):
    """Custom JSON encoder for Pydantic models and special types."""

    def default(self, obj):
        if isinstance(obj, UUID):
            return str(obj)
        if isinstance(obj, datetime):
            return obj.isoformat()
        if hasattr(obj, "model_dump"):
            return obj.model_dump()
        return super().default(obj)


class ValidationStorage:
    """Storage manager for validation results."""

    def __init__(self, base_dir: Optional[Path] = None):
        """Initialize storage with base directory.

        Args:
            base_dir: Base directory for validation data. If None, uses config default.
        """
        settings = get_settings()
        self.base_dir = base_dir or settings.storage.data_dir
        self.runs_dir = self.base_dir / "runs"
        self.history_dir = self.base_dir / "test-history"
        self.flaky_dir = self.base_dir / "flaky-tests"
        self._ensure_directories()

    def _ensure_directories(self) -> None:
        """Create required directories if they don't exist."""
        self.runs_dir.mkdir(parents=True, exist_ok=True)
        self.history_dir.mkdir(parents=True, exist_ok=True)
        self.flaky_dir.mkdir(parents=True, exist_ok=True)

    def _read_json(self, path: Path) -> Optional[dict]:
        """Read and parse a JSON file.

        Args:
            path: Path to JSON file

        Returns:
            Parsed JSON data or None if file doesn't exist
        """
        if not path.exists():
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except (json.JSONDecodeError, OSError):
            return None

    def _write_json(self, path: Path, data: dict) -> bool:
        """Write data to a JSON file.

        Args:
            path: Path to write to
            data: Data to serialize

        Returns:
            True if successful, False otherwise
        """
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(data, f, cls=JSONEncoder, indent=2)
            return True
        except OSError:
            return False

    # =========================================================================
    # Validation Run Operations
    # =========================================================================

    def save_run(self, run: ValidationRun) -> bool:
        """Save a validation run.

        Args:
            run: ValidationRun to save

        Returns:
            True if successful
        """
        run_path = self.runs_dir / f"{run.run_id}.json"
        success = self._write_json(run_path, run.model_dump())
        if success:
            self._update_runs_index(run)
        return success

    def get_run(self, run_id: UUID) -> Optional[ValidationRun]:
        """Retrieve a validation run by ID.

        Args:
            run_id: UUID of the run

        Returns:
            ValidationRun or None if not found
        """
        run_path = self.runs_dir / f"{run_id}.json"
        data = self._read_json(run_path)
        if data:
            return ValidationRun.model_validate(data)
        return None

    def get_latest_run(self) -> Optional[ValidationRun]:
        """Get the most recent validation run.

        Returns:
            Most recent ValidationRun or None
        """
        index = self._get_runs_index()
        if not index.get("runs"):
            return None
        latest = max(index["runs"], key=lambda x: x.get("timestamp", ""))
        return self.get_run(UUID(latest["run_id"]))

    def get_runs_by_branch(self, branch: str) -> list[ValidationRun]:
        """Get all runs for a specific branch.

        Args:
            branch: Branch name

        Returns:
            List of ValidationRun objects
        """
        index = self._get_runs_index()
        runs = []
        for entry in index.get("runs", []):
            if entry.get("branch") == branch:
                run = self.get_run(UUID(entry["run_id"]))
                if run:
                    runs.append(run)
        return runs

    def get_runs_by_status(
        self, status: ValidationStatus, limit: int = 100
    ) -> list[ValidationRun]:
        """Get runs filtered by status.

        Args:
            status: Status to filter by
            limit: Maximum number of runs to return

        Returns:
            List of ValidationRun objects
        """
        index = self._get_runs_index()
        runs = []
        for entry in index.get("runs", [])[:limit]:
            if entry.get("status") == status.value:
                run = self.get_run(UUID(entry["run_id"]))
                if run:
                    runs.append(run)
        return runs

    def delete_run(self, run_id: UUID) -> bool:
        """Delete a validation run.

        Args:
            run_id: UUID of the run to delete

        Returns:
            True if deleted successfully
        """
        run_path = self.runs_dir / f"{run_id}.json"
        if run_path.exists():
            run_path.unlink()
            self._remove_from_runs_index(run_id)
            return True
        return False

    def _get_runs_index(self) -> dict:
        """Get the runs index file."""
        index_path = self.runs_dir / "index.json"
        return self._read_json(index_path) or {"runs": []}

    def _update_runs_index(self, run: ValidationRun) -> None:
        """Update the runs index with a new/updated run."""
        index = self._get_runs_index()

        # Remove existing entry for this run if present
        index["runs"] = [r for r in index["runs"] if r.get("run_id") != str(run.run_id)]

        # Add new entry
        index["runs"].append(
            {
                "run_id": str(run.run_id),
                "timestamp": run.timestamp.isoformat(),
                "status": run.status.value,
                "branch": run.branch,
                "commit_sha": run.commit_sha,
                "repository": run.repository,
            }
        )

        # Sort by timestamp descending
        index["runs"].sort(key=lambda x: x.get("timestamp", ""), reverse=True)

        self._write_json(self.runs_dir / "index.json", index)

    def _remove_from_runs_index(self, run_id: UUID) -> None:
        """Remove a run from the index."""
        index = self._get_runs_index()
        index["runs"] = [r for r in index["runs"] if r.get("run_id") != str(run_id)]
        self._write_json(self.runs_dir / "index.json", index)

    # =========================================================================
    # Test Case History Operations
    # =========================================================================

    def save_test_case(self, test_case: TestCase) -> bool:
        """Save a test case with its execution history.

        Args:
            test_case: TestCase to save

        Returns:
            True if successful
        """
        # Create safe filename from test_id
        safe_id = test_case.test_id.replace("/", "_").replace("\\", "_").replace(":", "_")
        test_path = self.history_dir / f"{safe_id}.json"
        success = self._write_json(test_path, test_case.model_dump())
        if success:
            self._update_test_index(test_case)
        return success

    def get_test_case(self, test_id: str) -> Optional[TestCase]:
        """Retrieve a test case by ID.

        Args:
            test_id: Test identifier

        Returns:
            TestCase or None if not found
        """
        safe_id = test_id.replace("/", "_").replace("\\", "_").replace(":", "_")
        test_path = self.history_dir / f"{safe_id}.json"
        data = self._read_json(test_path)
        if data:
            return TestCase.model_validate(data)
        return None

    def get_all_test_cases(self) -> list[TestCase]:
        """Get all tracked test cases.

        Returns:
            List of TestCase objects
        """
        test_cases = []
        for path in self.history_dir.glob("*.json"):
            if path.name != "index.json":
                data = self._read_json(path)
                if data:
                    try:
                        test_cases.append(TestCase.model_validate(data))
                    except Exception:
                        continue
        return test_cases

    def _get_test_index(self) -> dict:
        """Get the test cases index file."""
        index_path = self.history_dir / "index.json"
        return self._read_json(index_path) or {"tests": []}

    def _update_test_index(self, test_case: TestCase) -> None:
        """Update the test cases index."""
        index = self._get_test_index()

        # Remove existing entry
        index["tests"] = [t for t in index["tests"] if t.get("test_id") != test_case.test_id]

        # Add new entry
        index["tests"].append(
            {
                "test_id": test_case.test_id,
                "test_name": test_case.test_name,
                "test_type": test_case.test_type.value,
                "is_flaky": test_case.is_flaky,
                "last_execution": (
                    test_case.last_execution.isoformat() if test_case.last_execution else None
                ),
            }
        )

        self._write_json(self.history_dir / "index.json", index)

    # =========================================================================
    # Flaky Test Operations
    # =========================================================================

    def save_flaky_test(self, record: FlakyTestRecord) -> bool:
        """Save a flaky test record.

        Args:
            record: FlakyTestRecord to save

        Returns:
            True if successful
        """
        safe_id = record.test_id.replace("/", "_").replace("\\", "_").replace(":", "_")
        record_path = self.flaky_dir / f"{safe_id}.json"
        success = self._write_json(record_path, record.model_dump())
        if success:
            self._update_flaky_index()
        return success

    def get_flaky_test(self, test_id: str) -> Optional[FlakyTestRecord]:
        """Retrieve a flaky test record.

        Args:
            test_id: Test identifier

        Returns:
            FlakyTestRecord or None
        """
        safe_id = test_id.replace("/", "_").replace("\\", "_").replace(":", "_")
        record_path = self.flaky_dir / f"{safe_id}.json"
        data = self._read_json(record_path)
        if data:
            return FlakyTestRecord.model_validate(data)
        return None

    def get_active_flaky_tests(self) -> list[FlakyTestRecord]:
        """Get all currently active flaky tests.

        Returns:
            List of active FlakyTestRecord objects
        """
        records = []
        for path in self.flaky_dir.glob("*.json"):
            if path.name not in ("index.json", "active-flaky.json"):
                data = self._read_json(path)
                if data and data.get("status") == "active_flaky":
                    try:
                        records.append(FlakyTestRecord.model_validate(data))
                    except Exception:
                        continue
        return records

    def get_all_flaky_tests(self) -> list[FlakyTestRecord]:
        """Get all flaky test records (active and resolved).

        Returns:
            List of all FlakyTestRecord objects
        """
        records = []
        for path in self.flaky_dir.glob("*.json"):
            if path.name not in ("index.json", "active-flaky.json"):
                data = self._read_json(path)
                if data:
                    try:
                        records.append(FlakyTestRecord.model_validate(data))
                    except Exception:
                        continue
        return records

    def _update_flaky_index(self) -> None:
        """Update the active flaky tests quick-lookup file."""
        active = self.get_active_flaky_tests()
        self._write_json(
            self.flaky_dir / "active-flaky.json",
            {
                "count": len(active),
                "test_ids": [r.test_id for r in active],
                "updated_at": datetime.utcnow().isoformat(),
            },
        )

    # =========================================================================
    # Cleanup Operations
    # =========================================================================

    def cleanup_old_runs(self, retention_days: Optional[int] = None) -> int:
        """Remove validation runs older than retention period.

        Args:
            retention_days: Days to retain (uses config default if None)

        Returns:
            Number of runs deleted
        """
        settings = get_settings()
        retention_days = retention_days or settings.storage.retention_days
        cutoff = datetime.utcnow() - timedelta(days=retention_days)

        deleted = 0
        index = self._get_runs_index()
        for entry in index.get("runs", []):
            timestamp_str = entry.get("timestamp")
            if timestamp_str:
                try:
                    timestamp = datetime.fromisoformat(timestamp_str)
                    if timestamp < cutoff:
                        if self.delete_run(UUID(entry["run_id"])):
                            deleted += 1
                except (ValueError, KeyError):
                    continue

        return deleted


    # =========================================================================
    # Generic Storage Operations (for backward compatibility)
    # =========================================================================

    def save(self, key: str, data: dict) -> bool:
        """Save data by key.

        Args:
            key: Storage key (path-like)
            data: Data to save

        Returns:
            True if successful
        """
        path = self.base_dir / f"{key}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        return self._write_json(path, data)

    def load(self, key: str) -> Optional[dict]:
        """Load data by key.

        Args:
            key: Storage key (path-like)

        Returns:
            Data or None if not found
        """
        path = self.base_dir / f"{key}.json"
        return self._read_json(path)

    def list(self, prefix: str = "") -> list[str]:
        """List keys with prefix.

        Args:
            prefix: Key prefix to filter by

        Returns:
            List of keys
        """
        search_dir = self.base_dir / prefix if prefix else self.base_dir
        if not search_dir.exists():
            return []

        keys = []
        for path in search_dir.glob("*.json"):
            if path.name not in ("index.json", "active-flaky.json"):
                key = str(path.relative_to(self.base_dir)).replace(".json", "")
                keys.append(key)
        return keys

    def list_runs(
        self,
        repository: Optional[str] = None,
        branch: Optional[str] = None,
        status: Optional[str] = None,
        limit: int = 20,
    ) -> list[ValidationRun]:
        """List validation runs with optional filters.

        Args:
            repository: Filter by repository
            branch: Filter by branch
            status: Filter by status
            limit: Maximum results

        Returns:
            List of ValidationRun objects
        """
        index = self._get_runs_index()
        runs = []

        for entry in index.get("runs", []):
            if repository and entry.get("repository") != repository:
                continue
            if branch and entry.get("branch") != branch:
                continue
            if status and entry.get("status") != status:
                continue

            run = self.get_run(UUID(entry["run_id"]))
            if run:
                runs.append(run)
                if len(runs) >= limit:
                    break

        return runs


# Alias for backward compatibility
Storage = ValidationStorage


# Singleton instance
_storage: Optional[ValidationStorage] = None


def get_storage() -> ValidationStorage:
    """Get the storage instance.

    Returns:
        ValidationStorage singleton
    """
    global _storage
    if _storage is None:
        _storage = ValidationStorage()
    return _storage
