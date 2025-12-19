"""Complexity analyzer service.

This module provides complexity analysis using Radon for Python code.
It calculates cyclomatic complexity and maintainability metrics.
"""

import json
import logging
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.core.exceptions import ComplexityAnalysisError
from src.quality.models import ComplexityResults


logger = logging.getLogger(__name__)


@dataclass
class FunctionComplexity:
    """Complexity data for a single function."""

    name: str
    file: str
    line: int
    complexity: int
    rank: str  # A, B, C, D, E, F


class ComplexityAnalyzer:
    """Service for analyzing code complexity."""

    def __init__(self):
        """Initialize the complexity analyzer."""
        self.settings = get_settings()
        self.max_complexity = self.settings.quality.max_cyclomatic_complexity

    def analyze(self, project_path: Path) -> ComplexityResults:
        """Analyze complexity of Python files in project.

        Args:
            project_path: Path to project root

        Returns:
            ComplexityResults with metrics

        Raises:
            ComplexityAnalysisError: If analysis fails
        """
        logger.info(f"Analyzing complexity for {project_path}")

        try:
            # Run radon cc (cyclomatic complexity)
            result = subprocess.run(
                [
                    "radon",
                    "cc",
                    str(project_path),
                    "-j",  # JSON output
                    "-a",  # Show average
                    "--total-average",
                ],
                capture_output=True,
                text=True,
                timeout=300,
            )

            functions = []
            total_complexity = 0
            function_count = 0
            max_complexity = 0
            high_complexity_functions = []

            if result.stdout:
                try:
                    radon_results = json.loads(result.stdout)

                    for file_path, file_data in radon_results.items():
                        if isinstance(file_data, list):
                            for item in file_data:
                                complexity = item.get("complexity", 0)
                                func_name = f"{file_path}::{item.get('name', 'unknown')}"

                                func = FunctionComplexity(
                                    name=item.get("name", "unknown"),
                                    file=file_path,
                                    line=item.get("lineno", 0),
                                    complexity=complexity,
                                    rank=item.get("rank", "A"),
                                )
                                functions.append(func)

                                total_complexity += complexity
                                function_count += 1
                                max_complexity = max(max_complexity, complexity)

                                if complexity > self.max_complexity:
                                    high_complexity_functions.append(func_name)

                except json.JSONDecodeError:
                    logger.warning("Failed to parse Radon JSON output")

            avg_complexity = (
                total_complexity / function_count if function_count > 0 else 0.0
            )

            return ComplexityResults(
                average_complexity=round(avg_complexity, 2),
                max_complexity=max_complexity,
                high_complexity_functions=high_complexity_functions,
            )

        except subprocess.TimeoutExpired:
            raise ComplexityAnalysisError("Complexity analysis timed out after 300s")
        except FileNotFoundError:
            raise ComplexityAnalysisError("Radon not installed. Run: pip install radon")
        except subprocess.SubprocessError as e:
            raise ComplexityAnalysisError(str(e))

    def get_maintainability_index(self, project_path: Path) -> float:
        """Calculate maintainability index for project.

        Args:
            project_path: Path to project root

        Returns:
            Maintainability index (0-100)

        Raises:
            ComplexityAnalysisError: If calculation fails
        """
        logger.info(f"Calculating maintainability index for {project_path}")

        try:
            result = subprocess.run(
                [
                    "radon",
                    "mi",
                    str(project_path),
                    "-j",  # JSON output
                ],
                capture_output=True,
                text=True,
                timeout=300,
            )

            total_mi = 0
            file_count = 0

            if result.stdout:
                try:
                    mi_results = json.loads(result.stdout)
                    for file_path, mi_data in mi_results.items():
                        if isinstance(mi_data, dict) and "mi" in mi_data:
                            total_mi += mi_data["mi"]
                            file_count += 1
                except json.JSONDecodeError:
                    logger.warning("Failed to parse Radon MI JSON output")

            if file_count == 0:
                return 100.0  # Perfect score if no files analyzed

            return round(total_mi / file_count, 2)

        except subprocess.TimeoutExpired:
            raise ComplexityAnalysisError("MI calculation timed out after 300s")
        except FileNotFoundError:
            raise ComplexityAnalysisError("Radon not installed. Run: pip install radon")
        except subprocess.SubprocessError as e:
            raise ComplexityAnalysisError(str(e))

    def analyze_file(self, file_path: Path) -> list[FunctionComplexity]:
        """Analyze complexity of a single file.

        Args:
            file_path: Path to file

        Returns:
            List of function complexity data
        """
        if not file_path.exists():
            return []

        try:
            result = subprocess.run(
                ["radon", "cc", str(file_path), "-j"],
                capture_output=True,
                text=True,
                timeout=60,
            )

            functions = []
            if result.stdout:
                try:
                    radon_results = json.loads(result.stdout)
                    for file_str, file_data in radon_results.items():
                        if isinstance(file_data, list):
                            for item in file_data:
                                functions.append(
                                    FunctionComplexity(
                                        name=item.get("name", "unknown"),
                                        file=file_str,
                                        line=item.get("lineno", 0),
                                        complexity=item.get("complexity", 0),
                                        rank=item.get("rank", "A"),
                                    )
                                )
                except json.JSONDecodeError:
                    pass

            return functions

        except (subprocess.SubprocessError, FileNotFoundError):
            return []


def get_complexity_analyzer() -> ComplexityAnalyzer:
    """Get complexity analyzer instance.

    Returns:
        ComplexityAnalyzer instance
    """
    return ComplexityAnalyzer()
