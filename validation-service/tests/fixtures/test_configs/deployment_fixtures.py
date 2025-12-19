"""Deployment configuration fixtures.

Provides test fixtures for deployment validation testing.
"""

import pytest
from pathlib import Path
from typing import Generator


@pytest.fixture
def deployable_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project that should pass deployment validation.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "deployable_project"
    project.mkdir()

    # Create proper .env file
    env_file = project / ".env"
    env_file.write_text("""
NODE_ENV=production
LOG_LEVEL=info
DATABASE_URL=postgres://localhost/app
REDIS_URL=redis://localhost:6379
""")

    # Create .env.example matching .env
    env_example = project / ".env.example"
    env_example.write_text("""
NODE_ENV=
LOG_LEVEL=
DATABASE_URL=
REDIS_URL=
""")

    # Create proper .gitignore
    gitignore = project / ".gitignore"
    gitignore.write_text("""
.env
*.pem
*.key
credentials
node_modules/
__pycache__/
""")

    # Create requirements.txt
    requirements = project / "requirements.txt"
    requirements.write_text("""
flask>=2.0.0
pydantic>=2.0.0
requests>=2.28.0
""")

    # Create secure code
    app = project / "app.py"
    app.write_text("""
import os
from flask import Flask

app = Flask(__name__)

DATABASE_URL = os.environ.get("DATABASE_URL")
DEBUG = os.environ.get("DEBUG", "false").lower() == "true"

@app.route("/health")
def health():
    return {"status": "healthy"}

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", 5000)))
""")

    yield project


@pytest.fixture
def undeployable_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a temporary project that should fail deployment validation.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "undeployable_project"
    project.mkdir()

    # Missing .env
    # No .env file created

    # Create .env.example with required vars
    env_example = project / ".env.example"
    env_example.write_text("""
DATABASE_URL=
API_KEY=
SECRET_KEY=
""")

    # Incomplete .gitignore
    gitignore = project / ".gitignore"
    gitignore.write_text("""
__pycache__/
""")
    # Missing .env, *.pem, credentials

    # Create insecure code
    app = project / "app.py"
    app.write_text("""
import pickle
import subprocess

# Hardcoded secrets
API_KEY = "sk-1234567890abcdef1234567890abcdef1234567890abcdef12"
DATABASE_PASSWORD = "super_secret_password_123"

DEBUG = True  # Debug enabled

def execute_command(cmd):
    subprocess.call(cmd, shell=True)  # Shell injection

def load_data(data):
    return pickle.loads(data)  # Unsafe pickle

def get_user(user_id):
    query = f"SELECT * FROM users WHERE id = {user_id}"  # SQL injection
    return query
""")

    yield project


@pytest.fixture
def missing_deps_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a project with dependency issues.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "missing_deps_project"
    project.mkdir()

    # Create requirements with conflicting versions
    requirements = project / "requirements.txt"
    requirements.write_text("""
flask>=3.0.0
werkzeug==1.0.0
# flask 3.0 requires werkzeug>=3.0.0, creating conflict
""")

    # Create minimal .env
    env = project / ".env"
    env.write_text("NODE_ENV=development")

    yield project


@pytest.fixture
def incompatible_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a project with platform compatibility issues.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "incompatible_project"
    project.mkdir()

    # Create .python-version requiring very new Python
    python_version = project / ".python-version"
    python_version.write_text("3.20")  # Future version

    # Create .nvmrc requiring very new Node
    nvmrc = project / ".nvmrc"
    nvmrc.write_text("v30.0.0")  # Future version

    # Create package.json with strict engine
    package = project / "package.json"
    package.write_text("""{
    "name": "test",
    "engines": {
        "node": ">=30.0.0"
    }
}""")

    yield project


@pytest.fixture
def docker_project(tmp_path: Path) -> Generator[Path, None, None]:
    """Create a project with Docker configuration.

    Yields:
        Path to the temporary project directory
    """
    project = tmp_path / "docker_project"
    project.mkdir()

    # Create Dockerfile
    dockerfile = project / "Dockerfile"
    dockerfile.write_text("""
FROM python:3.11-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY . .
CMD ["python", "app.py"]
""")

    # Create docker-compose.yml
    compose = project / "docker-compose.yml"
    compose.write_text("""
version: "3.8"
services:
  app:
    build: .
    ports:
      - "5000:5000"
    environment:
      - DATABASE_URL=postgres://db:5432/app
    depends_on:
      - db
      - redis

  db:
    image: postgres:15
    environment:
      - POSTGRES_PASSWORD=secret

  redis:
    image: redis:7
""")

    # Create requirements.txt
    requirements = project / "requirements.txt"
    requirements.write_text("flask>=2.0.0\nredis>=4.0.0")

    # Create app.py
    app = project / "app.py"
    app.write_text("""
import os
from flask import Flask

app = Flask(__name__)

@app.route("/")
def hello():
    return "Hello from Docker!"
""")

    yield project
