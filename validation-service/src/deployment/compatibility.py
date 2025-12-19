"""Compatibility checker service.

This module provides environment compatibility checking for deployment,
including resource requirements and platform validation.
"""

import logging
import os
import platform
import shutil
import subprocess
from pathlib import Path
from typing import Optional

from src.core.config import get_settings
from src.deployment.models import (
    ConfigCheckStatus,
    ResourceCheck,
    PlatformCheck,
    ServiceCheck,
    CompatibilityValidationResult,
)


logger = logging.getLogger(__name__)


class CompatibilityChecker:
    """Service for checking deployment compatibility."""

    def __init__(self):
        """Initialize the compatibility checker."""
        self.settings = get_settings()

    def check(
        self,
        project_path: Path,
        target_environment: Optional[str] = None,
        check_services: bool = True,
    ) -> CompatibilityValidationResult:
        """Check deployment compatibility.

        Args:
            project_path: Path to project root
            target_environment: Target environment (dev, staging, production)
            check_services: Whether to check external service availability

        Returns:
            CompatibilityValidationResult
        """
        result = CompatibilityValidationResult()

        # Check resource requirements
        resource_checks = self._check_resources(project_path)
        result.resource_checks.extend(resource_checks)

        # Check platform compatibility
        platform_checks = self._check_platform(project_path)
        result.platform_checks.extend(platform_checks)

        # Check external service dependencies
        if check_services:
            service_checks = self._check_services(project_path)
            result.service_checks.extend(service_checks)

        # Determine overall status
        resource_issues = any(
            r.status != ConfigCheckStatus.VALID for r in result.resource_checks
        )
        platform_issues = any(not p.is_compatible for p in result.platform_checks)
        service_issues = any(
            s.status != ConfigCheckStatus.VALID and s.is_required
            for s in result.service_checks
        )

        if resource_issues or platform_issues or service_issues:
            result.status = ConfigCheckStatus.INVALID
        else:
            result.status = ConfigCheckStatus.VALID

        return result

    def _check_resources(self, project_path: Path) -> list[ResourceCheck]:
        """Check resource requirements.

        Args:
            project_path: Path to project root

        Returns:
            List of ResourceCheck results
        """
        checks = []

        # Check disk space
        disk = shutil.disk_usage(project_path)
        free_gb = disk.free / (1024 ** 3)
        required_gb = 1.0  # Minimum 1GB free space

        checks.append(ResourceCheck(
            resource_type="disk_space",
            required=f"{required_gb}GB",
            available=f"{free_gb:.2f}GB",
            status=ConfigCheckStatus.VALID if free_gb >= required_gb else ConfigCheckStatus.WARNING,
            message="Sufficient disk space" if free_gb >= required_gb else "Low disk space",
        ))

        # Check memory (if we can determine requirements)
        try:
            import psutil
            mem = psutil.virtual_memory()
            available_gb = mem.available / (1024 ** 3)
            checks.append(ResourceCheck(
                resource_type="memory",
                required="512MB",
                available=f"{available_gb:.2f}GB",
                status=ConfigCheckStatus.VALID if available_gb >= 0.5 else ConfigCheckStatus.WARNING,
            ))
        except ImportError:
            checks.append(ResourceCheck(
                resource_type="memory",
                required="512MB",
                available="Unknown",
                status=ConfigCheckStatus.WARNING,
                message="Could not check memory (psutil not available)",
            ))

        # Check for required tools
        required_tools = self._detect_required_tools(project_path)
        for tool in required_tools:
            is_available = shutil.which(tool) is not None
            checks.append(ResourceCheck(
                resource_type=f"tool:{tool}",
                required="installed",
                available="installed" if is_available else "not found",
                status=ConfigCheckStatus.VALID if is_available else ConfigCheckStatus.INVALID,
                message=f"{tool} is {'available' if is_available else 'missing'}",
            ))

        return checks

    def _detect_required_tools(self, project_path: Path) -> list[str]:
        """Detect required tools based on project files.

        Args:
            project_path: Path to project root

        Returns:
            List of required tool names
        """
        tools = []

        if (project_path / "package.json").exists():
            tools.append("node")
            tools.append("npm")

        if (project_path / "requirements.txt").exists() or (project_path / "pyproject.toml").exists():
            tools.append("python")
            tools.append("pip")

        if (project_path / "Dockerfile").exists():
            tools.append("docker")

        if (project_path / "docker-compose.yml").exists() or (project_path / "docker-compose.yaml").exists():
            tools.append("docker-compose")

        return tools

    def _check_platform(self, project_path: Path) -> list[PlatformCheck]:
        """Check platform compatibility.

        Args:
            project_path: Path to project root

        Returns:
            List of PlatformCheck results
        """
        checks = []

        # Check Python version
        python_req = self._detect_python_version(project_path)
        if python_req:
            current = platform.python_version()
            is_compatible = self._compare_versions(current, python_req)
            checks.append(PlatformCheck(
                platform="python",
                required_version=python_req,
                detected_version=current,
                is_compatible=is_compatible,
                warnings=[] if is_compatible else [f"Python {python_req} required, found {current}"],
            ))

        # Check Node.js version
        node_req = self._detect_node_version(project_path)
        if node_req:
            try:
                result = subprocess.run(["node", "--version"], capture_output=True, text=True)
                current = result.stdout.strip().lstrip("v")
                is_compatible = self._compare_versions(current, node_req)
                checks.append(PlatformCheck(
                    platform="node",
                    required_version=node_req,
                    detected_version=current,
                    is_compatible=is_compatible,
                    warnings=[] if is_compatible else [f"Node {node_req} required, found {current}"],
                ))
            except FileNotFoundError:
                checks.append(PlatformCheck(
                    platform="node",
                    required_version=node_req,
                    detected_version="not installed",
                    is_compatible=False,
                    warnings=["Node.js is not installed"],
                ))

        # Check OS compatibility
        os_name = platform.system().lower()
        checks.append(PlatformCheck(
            platform="os",
            detected_version=f"{platform.system()} {platform.release()}",
            is_compatible=True,  # Most projects are cross-platform
        ))

        return checks

    def _detect_python_version(self, project_path: Path) -> Optional[str]:
        """Detect required Python version from project.

        Args:
            project_path: Path to project root

        Returns:
            Required Python version or None
        """
        # Check pyproject.toml
        pyproject = project_path / "pyproject.toml"
        if pyproject.exists():
            content = pyproject.read_text()
            import re
            match = re.search(r'python\s*[=<>]=?\s*["\']?([0-9.]+)', content)
            if match:
                return match.group(1)

        # Check .python-version
        python_version_file = project_path / ".python-version"
        if python_version_file.exists():
            return python_version_file.read_text().strip()

        # Check runtime.txt (Heroku style)
        runtime_file = project_path / "runtime.txt"
        if runtime_file.exists():
            content = runtime_file.read_text().strip()
            if content.startswith("python-"):
                return content.replace("python-", "")

        return None

    def _detect_node_version(self, project_path: Path) -> Optional[str]:
        """Detect required Node.js version from project.

        Args:
            project_path: Path to project root

        Returns:
            Required Node version or None
        """
        # Check .nvmrc
        nvmrc = project_path / ".nvmrc"
        if nvmrc.exists():
            return nvmrc.read_text().strip().lstrip("v")

        # Check package.json engines field
        package_json = project_path / "package.json"
        if package_json.exists():
            import json
            try:
                data = json.loads(package_json.read_text())
                engines = data.get("engines", {})
                node_req = engines.get("node", "")
                if node_req:
                    # Extract version number from constraint
                    import re
                    match = re.search(r"([0-9.]+)", node_req)
                    if match:
                        return match.group(1)
            except json.JSONDecodeError:
                pass

        return None

    def _compare_versions(self, current: str, required: str) -> bool:
        """Compare version strings.

        Args:
            current: Current version
            required: Required version

        Returns:
            True if current meets requirement
        """
        try:
            current_parts = [int(p) for p in current.split(".")[:3]]
            required_parts = [int(p) for p in required.split(".")[:3]]

            # Pad with zeros
            while len(current_parts) < 3:
                current_parts.append(0)
            while len(required_parts) < 3:
                required_parts.append(0)

            return current_parts >= required_parts
        except ValueError:
            return False

    def _check_services(self, project_path: Path) -> list[ServiceCheck]:
        """Check external service dependencies.

        Args:
            project_path: Path to project root

        Returns:
            List of ServiceCheck results
        """
        checks = []

        # Detect services from configuration
        services = self._detect_services(project_path)

        for service_name, endpoint, is_required in services:
            status, latency, error = self._check_service_connectivity(endpoint)
            checks.append(ServiceCheck(
                service_name=service_name,
                endpoint=endpoint,
                status=status,
                latency_ms=latency,
                error_message=error,
                is_required=is_required,
            ))

        return checks

    def _detect_services(self, project_path: Path) -> list[tuple[str, str, bool]]:
        """Detect external services from configuration.

        Args:
            project_path: Path to project root

        Returns:
            List of (service_name, endpoint, is_required) tuples
        """
        services = []

        # Check docker-compose for services
        for compose_file in ["docker-compose.yml", "docker-compose.yaml"]:
            compose_path = project_path / compose_file
            if compose_path.exists():
                try:
                    import yaml
                    data = yaml.safe_load(compose_path.read_text())
                    for service_name, config in data.get("services", {}).items():
                        if "image" in config:
                            # External dependency
                            image = config["image"]
                            if "postgres" in image or "mysql" in image:
                                services.append((
                                    f"database:{service_name}",
                                    "localhost:5432",
                                    True,
                                ))
                            elif "redis" in image:
                                services.append((
                                    f"cache:{service_name}",
                                    "localhost:6379",
                                    False,
                                ))
                except ImportError:
                    logger.debug("PyYAML not available for parsing docker-compose")
                except Exception as e:
                    logger.debug(f"Error parsing docker-compose: {e}")

        return services

    def _check_service_connectivity(
        self, endpoint: str
    ) -> tuple[ConfigCheckStatus, Optional[float], Optional[str]]:
        """Check if a service endpoint is reachable.

        Args:
            endpoint: Service endpoint (host:port)

        Returns:
            Tuple of (status, latency_ms, error_message)
        """
        import socket
        import time

        try:
            host, port = endpoint.rsplit(":", 1)
            port = int(port)

            start = time.time()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5)
            result = sock.connect_ex((host, port))
            latency = (time.time() - start) * 1000
            sock.close()

            if result == 0:
                return ConfigCheckStatus.VALID, latency, None
            else:
                return ConfigCheckStatus.INVALID, None, f"Connection refused on {endpoint}"

        except socket.timeout:
            return ConfigCheckStatus.INVALID, None, f"Connection timed out to {endpoint}"
        except Exception as e:
            return ConfigCheckStatus.INVALID, None, str(e)


def get_compatibility_checker() -> CompatibilityChecker:
    """Get compatibility checker instance.

    Returns:
        CompatibilityChecker instance
    """
    return CompatibilityChecker()
