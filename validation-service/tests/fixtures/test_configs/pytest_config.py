"""Pytest configuration fixtures.

Provides test fixtures for pytest-based test configurations.
"""

import pytest
from pathlib import Path
from typing import Generator


@pytest.fixture
def pytest_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project with pytest configuration.

    Yields:
        Path to the temporary project directory
    """
    # Create project structure
    project = tmp_path / "pytest_project"
    project.mkdir()

    # Create pytest.ini
    pytest_ini = project / "pytest.ini"
    pytest_ini.write_text("""[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
markers =
    unit: Unit tests
    integration: Integration tests
    e2e: End-to-end tests
    slow: Slow tests
""")

    # Create pyproject.toml with pytest config
    pyproject = project / "pyproject.toml"
    pyproject.write_text("""[project]
name = "test-project"
version = "1.0.0"

[tool.pytest.ini_options]
addopts = "-v"
testpaths = ["tests"]
""")

    # Create test directories
    tests_dir = project / "tests"
    tests_dir.mkdir()

    unit_dir = tests_dir / "unit"
    unit_dir.mkdir()

    integration_dir = tests_dir / "integration"
    integration_dir.mkdir()

    e2e_dir = tests_dir / "e2e"
    e2e_dir.mkdir()

    # Create conftest.py
    conftest = tests_dir / "conftest.py"
    conftest.write_text("""import pytest

@pytest.fixture
def sample_data():
    return {"key": "value"}
""")

    # Create unit tests
    unit_test = unit_dir / "test_sample.py"
    unit_test.write_text("""import pytest

@pytest.mark.unit
def test_addition():
    assert 1 + 1 == 2

@pytest.mark.unit
def test_subtraction():
    assert 5 - 3 == 2

@pytest.mark.unit
def test_with_fixture(sample_data):
    assert sample_data["key"] == "value"
""")

    # Create integration tests
    integration_test = integration_dir / "test_integration.py"
    integration_test.write_text("""import pytest

@pytest.mark.integration
def test_database_connection():
    # Simulated integration test
    assert True

@pytest.mark.integration
def test_api_endpoint():
    # Simulated integration test
    assert True
""")

    # Create E2E tests
    e2e_test = e2e_dir / "test_user_flow.py"
    e2e_test.write_text("""import pytest

@pytest.mark.e2e
def test_login_flow():
    # Simulated E2E test
    assert True

@pytest.mark.e2e
@pytest.mark.slow
def test_full_workflow():
    # Simulated slow E2E test
    assert True
""")

    yield project


@pytest.fixture
def failing_tests_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project with failing tests.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "failing_project"
    project.mkdir()

    tests_dir = project / "tests"
    tests_dir.mkdir()

    # Create tests with failures
    test_file = tests_dir / "test_failures.py"
    test_file.write_text("""import pytest

def test_passing():
    assert True

def test_failing():
    assert False, "This test intentionally fails"

def test_error():
    raise RuntimeError("This test raises an error")

@pytest.mark.skip(reason="Skipped for testing")
def test_skipped():
    assert True
""")

    yield project


@pytest.fixture
def flaky_tests_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project with flaky tests.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "flaky_project"
    project.mkdir()

    tests_dir = project / "tests"
    tests_dir.mkdir()

    # Create tests that might fail intermittently
    test_file = tests_dir / "test_flaky.py"
    test_file.write_text("""import pytest
import random

def test_random_failure():
    # 50% chance of failure
    assert random.random() > 0.5

def test_timing_dependent():
    import time
    start = time.time()
    time.sleep(0.01)
    elapsed = time.time() - start
    # Might fail on slow systems
    assert elapsed < 0.1

def test_stable():
    # Always passes
    assert True
""")

    yield project


@pytest.fixture
def empty_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project with no tests.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "empty_project"
    project.mkdir()

    # Create minimal structure
    tests_dir = project / "tests"
    tests_dir.mkdir()

    conftest = tests_dir / "conftest.py"
    conftest.write_text("# Empty conftest")

    yield project
