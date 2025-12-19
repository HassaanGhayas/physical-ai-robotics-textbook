"""Tests for the complexity analyzer service."""

import json
import pytest
from unittest.mock import Mock, patch
from pathlib import Path

from src.core.exceptions import ComplexityAnalysisError
from src.quality.complexity import (
    ComplexityAnalyzer,
    FunctionComplexity,
    get_complexity_analyzer,
)


class TestComplexityAnalyzer:
    """Tests for ComplexityAnalyzer."""

    def test_init(self):
        """Test complexity analyzer initialization."""
        analyzer = ComplexityAnalyzer()
        assert analyzer.settings is not None
        assert analyzer.max_complexity > 0

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_success_no_functions(self, mock_run):
        """Test analyzing project with no functions."""
        mock_run.return_value = Mock(
            stdout="{}",
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        result = analyzer.analyze(Path("/test/project"))

        assert result.average_complexity == 0.0
        assert result.max_complexity == 0
        assert len(result.high_complexity_functions) == 0

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_success_with_functions(self, mock_run):
        """Test analyzing project with functions."""
        radon_output = json.dumps({
            "src/main.py": [
                {
                    "name": "process_data",
                    "lineno": 10,
                    "complexity": 5,
                    "rank": "B",
                },
                {
                    "name": "complex_function",
                    "lineno": 50,
                    "complexity": 15,
                    "rank": "D",
                },
            ],
            "src/utils.py": [
                {
                    "name": "helper",
                    "lineno": 5,
                    "complexity": 2,
                    "rank": "A",
                },
            ],
        })

        mock_run.return_value = Mock(
            stdout=radon_output,
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        analyzer.max_complexity = 10  # Set threshold for testing
        result = analyzer.analyze(Path("/test/project"))

        # Average: (5 + 15 + 2) / 3 = 7.33
        assert result.average_complexity == 7.33
        assert result.max_complexity == 15
        # Only complex_function exceeds threshold of 10
        assert len(result.high_complexity_functions) == 1
        assert "complex_function" in result.high_complexity_functions[0]

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_invalid_json(self, mock_run):
        """Test handling of invalid JSON output from Radon."""
        mock_run.return_value = Mock(
            stdout="invalid json",
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        result = analyzer.analyze(Path("/test/project"))

        # Should return zero values when JSON parsing fails
        assert result.average_complexity == 0.0
        assert result.max_complexity == 0

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_timeout(self, mock_run):
        """Test handling of timeout."""
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired("radon", 300)

        analyzer = ComplexityAnalyzer()
        with pytest.raises(ComplexityAnalysisError) as exc_info:
            analyzer.analyze(Path("/test/project"))

        assert "timed out" in str(exc_info.value).lower()

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_not_installed(self, mock_run):
        """Test handling when Radon is not installed."""
        mock_run.side_effect = FileNotFoundError()

        analyzer = ComplexityAnalyzer()
        with pytest.raises(ComplexityAnalysisError) as exc_info:
            analyzer.analyze(Path("/test/project"))

        assert "not installed" in str(exc_info.value).lower()

    @patch("src.quality.complexity.subprocess.run")
    def test_get_maintainability_index_success(self, mock_run):
        """Test getting maintainability index."""
        mi_output = json.dumps({
            "src/main.py": {"mi": 75.5, "rank": "A"},
            "src/utils.py": {"mi": 85.0, "rank": "A"},
        })

        mock_run.return_value = Mock(
            stdout=mi_output,
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        mi = analyzer.get_maintainability_index(Path("/test/project"))

        # Average: (75.5 + 85.0) / 2 = 80.25
        assert mi == 80.25

    @patch("src.quality.complexity.subprocess.run")
    def test_get_maintainability_index_empty(self, mock_run):
        """Test maintainability index with no files."""
        mock_run.return_value = Mock(
            stdout="{}",
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        mi = analyzer.get_maintainability_index(Path("/test/project"))

        # Should return 100 if no files analyzed
        assert mi == 100.0

    @patch("src.quality.complexity.subprocess.run")
    def test_get_maintainability_index_timeout(self, mock_run):
        """Test maintainability index timeout handling."""
        import subprocess
        mock_run.side_effect = subprocess.TimeoutExpired("radon", 300)

        analyzer = ComplexityAnalyzer()
        with pytest.raises(ComplexityAnalysisError) as exc_info:
            analyzer.get_maintainability_index(Path("/test/project"))

        assert "timed out" in str(exc_info.value).lower()

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_file_existing(self, mock_run):
        """Test analyzing a single existing file."""
        radon_output = json.dumps({
            "/test/file.py": [
                {
                    "name": "my_function",
                    "lineno": 10,
                    "complexity": 5,
                    "rank": "B",
                },
            ],
        })

        mock_run.return_value = Mock(
            stdout=radon_output,
            stderr="",
            returncode=0,
        )

        analyzer = ComplexityAnalyzer()
        with patch.object(Path, "exists", return_value=True):
            functions = analyzer.analyze_file(Path("/test/file.py"))

        assert len(functions) == 1
        assert functions[0].name == "my_function"
        assert functions[0].complexity == 5
        assert functions[0].rank == "B"

    def test_analyze_file_not_existing(self):
        """Test analyzing a non-existing file."""
        analyzer = ComplexityAnalyzer()
        with patch.object(Path, "exists", return_value=False):
            functions = analyzer.analyze_file(Path("/test/nonexistent.py"))

        assert len(functions) == 0

    @patch("src.quality.complexity.subprocess.run")
    def test_analyze_file_subprocess_error(self, mock_run):
        """Test analyzing file with subprocess error."""
        import subprocess
        mock_run.side_effect = subprocess.SubprocessError()

        analyzer = ComplexityAnalyzer()
        with patch.object(Path, "exists", return_value=True):
            functions = analyzer.analyze_file(Path("/test/file.py"))

        # Should return empty list on error
        assert len(functions) == 0


class TestFunctionComplexity:
    """Tests for FunctionComplexity dataclass."""

    def test_create_function_complexity(self):
        """Test creating FunctionComplexity."""
        func = FunctionComplexity(
            name="my_function",
            file="src/main.py",
            line=10,
            complexity=5,
            rank="B",
        )

        assert func.name == "my_function"
        assert func.file == "src/main.py"
        assert func.line == 10
        assert func.complexity == 5
        assert func.rank == "B"


class TestGetComplexityAnalyzer:
    """Tests for get_complexity_analyzer function."""

    def test_get_complexity_analyzer(self):
        """Test getting complexity analyzer instance."""
        analyzer = get_complexity_analyzer()
        assert isinstance(analyzer, ComplexityAnalyzer)
