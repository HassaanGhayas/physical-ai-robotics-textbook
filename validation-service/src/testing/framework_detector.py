"""Test framework detection service.

This module provides automatic detection of test frameworks used in a project.
"""

import logging
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

from src.core.config import get_settings


logger = logging.getLogger(__name__)


class TestFramework(str, Enum):
    """Supported test frameworks."""

    PYTEST = "pytest"
    UNITTEST = "unittest"
    NOSE = "nose"
    PLAYWRIGHT = "playwright"
    SELENIUM = "selenium"
    CYPRESS = "cypress"
    JEST = "jest"
    MOCHA = "mocha"
    VITEST = "vitest"
    UNKNOWN = "unknown"


@dataclass
class FrameworkDetectionResult:
    """Result of framework detection."""

    framework: TestFramework
    confidence: float  # 0.0 to 1.0
    config_file: Optional[str] = None
    version: Optional[str] = None
    markers: list[str] = field(default_factory=list)


@dataclass
class ProjectTestConfig:
    """Test configuration detected for a project."""

    unit_framework: Optional[FrameworkDetectionResult] = None
    integration_framework: Optional[FrameworkDetectionResult] = None
    e2e_framework: Optional[FrameworkDetectionResult] = None
    test_directories: list[str] = field(default_factory=list)
    config_files: list[str] = field(default_factory=list)


class TestFrameworkDetector:
    """Service for detecting test frameworks in a project."""

    # Framework detection patterns
    FRAMEWORK_PATTERNS = {
        TestFramework.PYTEST: {
            "files": ["pytest.ini", "pyproject.toml", "setup.cfg", "conftest.py"],
            "imports": ["import pytest", "from pytest"],
            "markers": ["@pytest.fixture", "@pytest.mark"],
        },
        TestFramework.UNITTEST: {
            "files": [],
            "imports": ["import unittest", "from unittest"],
            "markers": ["unittest.TestCase", "self.assert"],
        },
        TestFramework.PLAYWRIGHT: {
            "files": ["playwright.config.ts", "playwright.config.js"],
            "imports": ["from playwright", "import { test }"],
            "markers": ["@playwright/test", "page.goto"],
        },
        TestFramework.SELENIUM: {
            "files": [],
            "imports": ["from selenium", "import selenium"],
            "markers": ["webdriver.", "find_element"],
        },
        TestFramework.JEST: {
            "files": ["jest.config.js", "jest.config.ts", "jest.config.json"],
            "imports": [],
            "markers": ["describe(", "it(", "expect("],
        },
        TestFramework.VITEST: {
            "files": ["vitest.config.ts", "vitest.config.js"],
            "imports": ["from 'vitest'", 'from "vitest"'],
            "markers": ["describe(", "it(", "expect("],
        },
        TestFramework.CYPRESS: {
            "files": ["cypress.config.js", "cypress.config.ts", "cypress.json"],
            "imports": [],
            "markers": ["cy.", "Cypress."],
        },
    }

    def __init__(self):
        """Initialize the framework detector."""
        self.settings = get_settings()

    def detect(self, project_path: Path) -> ProjectTestConfig:
        """Detect test frameworks in a project.

        Args:
            project_path: Path to project root

        Returns:
            ProjectTestConfig with detected frameworks
        """
        config = ProjectTestConfig()

        # Find test directories
        config.test_directories = self._find_test_directories(project_path)

        # Detect Python frameworks
        python_framework = self._detect_python_framework(project_path)
        if python_framework:
            config.unit_framework = python_framework
            config.integration_framework = python_framework

        # Detect E2E frameworks
        e2e_framework = self._detect_e2e_framework(project_path)
        if e2e_framework:
            config.e2e_framework = e2e_framework

        # Detect JavaScript frameworks (for frontend projects)
        js_framework = self._detect_js_framework(project_path)
        if js_framework and not config.unit_framework:
            config.unit_framework = js_framework

        # Collect config files
        config.config_files = self._find_config_files(project_path)

        return config

    def _find_test_directories(self, project_path: Path) -> list[str]:
        """Find test directories in the project.

        Args:
            project_path: Path to project root

        Returns:
            List of test directory paths
        """
        test_dirs = []
        common_names = ["tests", "test", "spec", "specs", "__tests__", "e2e"]

        for name in common_names:
            test_dir = project_path / name
            if test_dir.is_dir():
                test_dirs.append(str(test_dir.relative_to(project_path)))

        return test_dirs

    def _detect_python_framework(
        self, project_path: Path
    ) -> Optional[FrameworkDetectionResult]:
        """Detect Python test framework.

        Args:
            project_path: Path to project root

        Returns:
            FrameworkDetectionResult or None
        """
        # Check for pytest first (most common)
        pytest_patterns = self.FRAMEWORK_PATTERNS[TestFramework.PYTEST]

        for config_file in pytest_patterns["files"]:
            if (project_path / config_file).exists():
                return FrameworkDetectionResult(
                    framework=TestFramework.PYTEST,
                    confidence=1.0,
                    config_file=config_file,
                    markers=pytest_patterns["markers"],
                )

        # Check pyproject.toml for pytest config
        pyproject = project_path / "pyproject.toml"
        if pyproject.exists():
            try:
                content = pyproject.read_text()
                if "[tool.pytest" in content:
                    return FrameworkDetectionResult(
                        framework=TestFramework.PYTEST,
                        confidence=1.0,
                        config_file="pyproject.toml",
                        markers=pytest_patterns["markers"],
                    )
            except Exception:
                pass

        # Check requirements for pytest
        for req_file in ["requirements.txt", "requirements-dev.txt", "dev-requirements.txt"]:
            req_path = project_path / req_file
            if req_path.exists():
                try:
                    content = req_path.read_text().lower()
                    if "pytest" in content:
                        return FrameworkDetectionResult(
                            framework=TestFramework.PYTEST,
                            confidence=0.9,
                            config_file=req_file,
                            markers=pytest_patterns["markers"],
                        )
                except Exception:
                    pass

        # Default to pytest if tests directory exists with conftest
        tests_dir = project_path / "tests"
        if tests_dir.is_dir() and (tests_dir / "conftest.py").exists():
            return FrameworkDetectionResult(
                framework=TestFramework.PYTEST,
                confidence=0.8,
                config_file="tests/conftest.py",
            )

        # Check for unittest patterns
        return self._scan_for_unittest(project_path)

    def _scan_for_unittest(
        self, project_path: Path
    ) -> Optional[FrameworkDetectionResult]:
        """Scan for unittest usage in test files.

        Args:
            project_path: Path to project root

        Returns:
            FrameworkDetectionResult or None
        """
        unittest_patterns = self.FRAMEWORK_PATTERNS[TestFramework.UNITTEST]

        tests_dir = project_path / "tests"
        if not tests_dir.is_dir():
            return None

        try:
            for test_file in tests_dir.rglob("test_*.py"):
                content = test_file.read_text()
                for pattern in unittest_patterns["imports"]:
                    if pattern in content:
                        return FrameworkDetectionResult(
                            framework=TestFramework.UNITTEST,
                            confidence=0.7,
                            markers=unittest_patterns["markers"],
                        )
        except Exception:
            pass

        return None

    def _detect_e2e_framework(
        self, project_path: Path
    ) -> Optional[FrameworkDetectionResult]:
        """Detect E2E test framework.

        Args:
            project_path: Path to project root

        Returns:
            FrameworkDetectionResult or None
        """
        # Check for Playwright
        playwright_patterns = self.FRAMEWORK_PATTERNS[TestFramework.PLAYWRIGHT]
        for config_file in playwright_patterns["files"]:
            if (project_path / config_file).exists():
                return FrameworkDetectionResult(
                    framework=TestFramework.PLAYWRIGHT,
                    confidence=1.0,
                    config_file=config_file,
                    markers=playwright_patterns["markers"],
                )

        # Check for Cypress
        cypress_patterns = self.FRAMEWORK_PATTERNS[TestFramework.CYPRESS]
        for config_file in cypress_patterns["files"]:
            if (project_path / config_file).exists():
                return FrameworkDetectionResult(
                    framework=TestFramework.CYPRESS,
                    confidence=1.0,
                    config_file=config_file,
                    markers=cypress_patterns["markers"],
                )

        # Check for Selenium in Python
        selenium_patterns = self.FRAMEWORK_PATTERNS[TestFramework.SELENIUM]
        e2e_dir = project_path / "tests" / "e2e"
        if e2e_dir.is_dir():
            try:
                for test_file in e2e_dir.rglob("*.py"):
                    content = test_file.read_text()
                    for pattern in selenium_patterns["imports"]:
                        if pattern in content:
                            return FrameworkDetectionResult(
                                framework=TestFramework.SELENIUM,
                                confidence=0.8,
                                markers=selenium_patterns["markers"],
                            )
            except Exception:
                pass

        return None

    def _detect_js_framework(
        self, project_path: Path
    ) -> Optional[FrameworkDetectionResult]:
        """Detect JavaScript test framework.

        Args:
            project_path: Path to project root

        Returns:
            FrameworkDetectionResult or None
        """
        # Check for Jest
        jest_patterns = self.FRAMEWORK_PATTERNS[TestFramework.JEST]
        for config_file in jest_patterns["files"]:
            if (project_path / config_file).exists():
                return FrameworkDetectionResult(
                    framework=TestFramework.JEST,
                    confidence=1.0,
                    config_file=config_file,
                    markers=jest_patterns["markers"],
                )

        # Check for Vitest
        vitest_patterns = self.FRAMEWORK_PATTERNS[TestFramework.VITEST]
        for config_file in vitest_patterns["files"]:
            if (project_path / config_file).exists():
                return FrameworkDetectionResult(
                    framework=TestFramework.VITEST,
                    confidence=1.0,
                    config_file=config_file,
                    markers=vitest_patterns["markers"],
                )

        # Check package.json for test scripts
        package_json = project_path / "package.json"
        if package_json.exists():
            try:
                import json

                data = json.loads(package_json.read_text())
                deps = {
                    **data.get("dependencies", {}),
                    **data.get("devDependencies", {}),
                }

                if "vitest" in deps:
                    return FrameworkDetectionResult(
                        framework=TestFramework.VITEST,
                        confidence=0.9,
                        config_file="package.json",
                    )
                if "jest" in deps:
                    return FrameworkDetectionResult(
                        framework=TestFramework.JEST,
                        confidence=0.9,
                        config_file="package.json",
                    )
            except Exception:
                pass

        return None

    def _find_config_files(self, project_path: Path) -> list[str]:
        """Find all test configuration files.

        Args:
            project_path: Path to project root

        Returns:
            List of config file paths
        """
        config_files = []
        all_config_files = set()

        for framework_patterns in self.FRAMEWORK_PATTERNS.values():
            all_config_files.update(framework_patterns["files"])

        for config_file in all_config_files:
            if (project_path / config_file).exists():
                config_files.append(config_file)

        return config_files

    def get_runner_command(
        self,
        framework: TestFramework,
        test_path: Optional[str] = None,
    ) -> list[str]:
        """Get the command to run tests for a framework.

        Args:
            framework: Test framework
            test_path: Optional specific test path

        Returns:
            Command as list of strings
        """
        commands = {
            TestFramework.PYTEST: ["pytest", "-v"],
            TestFramework.UNITTEST: ["python", "-m", "unittest", "discover"],
            TestFramework.PLAYWRIGHT: ["playwright", "test"],
            TestFramework.JEST: ["npx", "jest"],
            TestFramework.VITEST: ["npx", "vitest", "run"],
            TestFramework.CYPRESS: ["npx", "cypress", "run"],
        }

        cmd = commands.get(framework, ["pytest", "-v"])

        if test_path:
            cmd.append(test_path)

        return cmd


def get_framework_detector() -> TestFrameworkDetector:
    """Get test framework detector instance.

    Returns:
        TestFrameworkDetector instance
    """
    return TestFrameworkDetector()
