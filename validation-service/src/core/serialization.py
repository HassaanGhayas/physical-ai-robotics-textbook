"""Serialization utilities for validation results.

This module provides functions for serializing and deserializing
validation models to/from JSON format.
"""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Optional, TypeVar, Union
from uuid import UUID

from pydantic import BaseModel

from src.core.models import (
    CodeQualityReport,
    DeploymentChecklist,
    FlakyTestRecord,
    TestCase,
    TestSuiteResult,
    ValidationRun,
)


T = TypeVar("T", bound=BaseModel)


class ValidationJSONEncoder(json.JSONEncoder):
    """Custom JSON encoder for validation models."""

    def default(self, obj: Any) -> Any:
        """Handle special types during JSON encoding.

        Args:
            obj: Object to encode

        Returns:
            JSON-serializable representation
        """
        if isinstance(obj, UUID):
            return str(obj)
        if isinstance(obj, datetime):
            return obj.isoformat()
        if isinstance(obj, Path):
            return str(obj)
        if hasattr(obj, "model_dump"):
            return obj.model_dump()
        if hasattr(obj, "value"):  # Enum
            return obj.value
        return super().default(obj)


def serialize_model(model: BaseModel) -> str:
    """Serialize a Pydantic model to JSON string.

    Args:
        model: Pydantic model instance

    Returns:
        JSON string representation
    """
    return json.dumps(model.model_dump(), cls=ValidationJSONEncoder, indent=2)


def serialize_model_to_dict(model: BaseModel) -> dict:
    """Serialize a Pydantic model to dictionary.

    Args:
        model: Pydantic model instance

    Returns:
        Dictionary representation
    """
    return json.loads(serialize_model(model))


def deserialize_model(model_class: type[T], data: Union[str, dict]) -> T:
    """Deserialize JSON data to a Pydantic model.

    Args:
        model_class: Target Pydantic model class
        data: JSON string or dictionary

    Returns:
        Model instance

    Raises:
        ValueError: If deserialization fails
    """
    if isinstance(data, str):
        data = json.loads(data)
    return model_class.model_validate(data)


def serialize_validation_run(run: ValidationRun) -> str:
    """Serialize a ValidationRun to JSON string.

    Args:
        run: ValidationRun instance

    Returns:
        JSON string
    """
    return serialize_model(run)


def deserialize_validation_run(data: Union[str, dict]) -> ValidationRun:
    """Deserialize JSON to a ValidationRun.

    Args:
        data: JSON string or dictionary

    Returns:
        ValidationRun instance
    """
    return deserialize_model(ValidationRun, data)


def serialize_quality_report(report: CodeQualityReport) -> str:
    """Serialize a CodeQualityReport to JSON string.

    Args:
        report: CodeQualityReport instance

    Returns:
        JSON string
    """
    return serialize_model(report)


def deserialize_quality_report(data: Union[str, dict]) -> CodeQualityReport:
    """Deserialize JSON to a CodeQualityReport.

    Args:
        data: JSON string or dictionary

    Returns:
        CodeQualityReport instance
    """
    return deserialize_model(CodeQualityReport, data)


def serialize_test_result(result: TestSuiteResult) -> str:
    """Serialize a TestSuiteResult to JSON string.

    Args:
        result: TestSuiteResult instance

    Returns:
        JSON string
    """
    return serialize_model(result)


def deserialize_test_result(data: Union[str, dict]) -> TestSuiteResult:
    """Deserialize JSON to a TestSuiteResult.

    Args:
        data: JSON string or dictionary

    Returns:
        TestSuiteResult instance
    """
    return deserialize_model(TestSuiteResult, data)


def serialize_deployment_checklist(checklist: DeploymentChecklist) -> str:
    """Serialize a DeploymentChecklist to JSON string.

    Args:
        checklist: DeploymentChecklist instance

    Returns:
        JSON string
    """
    return serialize_model(checklist)


def deserialize_deployment_checklist(data: Union[str, dict]) -> DeploymentChecklist:
    """Deserialize JSON to a DeploymentChecklist.

    Args:
        data: JSON string or dictionary

    Returns:
        DeploymentChecklist instance
    """
    return deserialize_model(DeploymentChecklist, data)


def serialize_test_case(test_case: TestCase) -> str:
    """Serialize a TestCase to JSON string.

    Args:
        test_case: TestCase instance

    Returns:
        JSON string
    """
    return serialize_model(test_case)


def deserialize_test_case(data: Union[str, dict]) -> TestCase:
    """Deserialize JSON to a TestCase.

    Args:
        data: JSON string or dictionary

    Returns:
        TestCase instance
    """
    return deserialize_model(TestCase, data)


def serialize_flaky_record(record: FlakyTestRecord) -> str:
    """Serialize a FlakyTestRecord to JSON string.

    Args:
        record: FlakyTestRecord instance

    Returns:
        JSON string
    """
    return serialize_model(record)


def deserialize_flaky_record(data: Union[str, dict]) -> FlakyTestRecord:
    """Deserialize JSON to a FlakyTestRecord.

    Args:
        data: JSON string or dictionary

    Returns:
        FlakyTestRecord instance
    """
    return deserialize_model(FlakyTestRecord, data)


def save_model_to_file(model: BaseModel, path: Path) -> bool:
    """Save a Pydantic model to a JSON file.

    Args:
        model: Model instance to save
        path: File path

    Returns:
        True if successful
    """
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(serialize_model(model))
        return True
    except (OSError, IOError):
        return False


def load_model_from_file(model_class: type[T], path: Path) -> Optional[T]:
    """Load a Pydantic model from a JSON file.

    Args:
        model_class: Target model class
        path: File path

    Returns:
        Model instance or None if file doesn't exist or is invalid
    """
    try:
        if not path.exists():
            return None
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        return model_class.model_validate(data)
    except (OSError, IOError, json.JSONDecodeError, ValueError):
        return None


def merge_validation_results(
    base: ValidationRun,
    quality_report: Optional[CodeQualityReport] = None,
    test_results: Optional[list[TestSuiteResult]] = None,
    deployment_checklist: Optional[DeploymentChecklist] = None,
) -> ValidationRun:
    """Merge individual results into a validation run.

    Args:
        base: Base ValidationRun
        quality_report: Optional quality report to add
        test_results: Optional test results to add
        deployment_checklist: Optional deployment checklist to add

    Returns:
        Updated ValidationRun with merged results
    """
    if quality_report:
        base.quality_report = quality_report

    if test_results:
        # Merge by replacing existing results with same suite_type
        existing_types = {r.suite_type for r in base.test_results}
        for result in test_results:
            if result.suite_type in existing_types:
                base.test_results = [
                    r if r.suite_type != result.suite_type else result
                    for r in base.test_results
                ]
            else:
                base.test_results.append(result)

    if deployment_checklist:
        base.deployment_checklist = deployment_checklist

    return base
