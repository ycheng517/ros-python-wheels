import socket
import subprocess
from dataclasses import dataclass
from enum import Enum

import requests
from rosdistro import get_distribution, get_index, get_index_url
from rosdistro.dependency_walker import DependencyWalker

IGNORE_LIST = [
    "python3-dev",
    "python3",
    "rmw_connextdds",
    "rmw_fastrtps_dynamic_cpp",
    "rmw_cyclonedds_cpp",
]


class PackageSource(Enum):
    ROS = 1
    PYTHON = 2
    SYSTEM = 3


@dataclass(frozen=True)
class Dependency:
    ros_name: str
    source: PackageSource
    # name for the dependency, if it's a Python or system package then
    # it could be different from ros_name
    dep_name: str


DEPENDENCY_MAP: dict[str, Dependency] = {
    "pybind11-dev":
    Dependency(
        ros_name="pybind11-dev",
        source=PackageSource.PYTHON,
        dep_name="pybind11",
    ),
}


def get_distribution_with_timeout(index, dist_name, timeout=30):
    """
    Wrapper around rosdistro.get_distribution that sets a timeout.
    """
    original_timeout = socket.getdefaulttimeout()
    socket.setdefaulttimeout(timeout)
    try:
        return get_distribution(index, dist_name)
    finally:
        socket.setdefaulttimeout(original_timeout)


class Distro:

    def __init__(self, distro_name):
        self.distro_name = distro_name
        self.index = get_index(get_index_url())
        self.distro_file = get_distribution_with_timeout(
            self.index, self.distro_name)
        self.dependency_walker = DependencyWalker(self.distro_file)

    def _check_pypi_package_exists(self, package_name):
        """
        Check if a package exists on PyPI.
        Returns True if the package exists, False otherwise.
        """
        try:
            response = requests.get(
                f"https://pypi.org/pypi/{package_name}/json", timeout=10)
            return response.status_code == 200
        except requests.RequestException:
            return False

    def _determine_package_source_and_name(self, package_name):
        """
        Determine if a package is a ROS, Python, or system package.
        Returns a tuple of (PackageSource, actual_package_name).
        """
        if package_name in self.distro_file.release_packages:
            return PackageSource.ROS, package_name
        else:
            # Check if it's a Python package by looking for python3- prefix
            if package_name.startswith("python3-"):
                # First try resolving with macOS to see if it's available via pip
                try:
                    result_macos = subprocess.run(
                        [
                            "rosdep",
                            "resolve",
                            "--os=osx:monterey",
                            package_name,
                        ],
                        capture_output=True,
                        text=True,
                        check=True,
                    )
                    lines_macos = result_macos.stdout.strip().split("\n")
                    if len(lines_macos) >= 2 and lines_macos[0].startswith(
                            "#"):
                        package_manager_macos = lines_macos[0].strip(
                            "#").strip()
                        actual_package_name_macos = lines_macos[1].strip()

                        # If macOS uses pip, prefer it as a Python package
                        if package_manager_macos == "pip":
                            return PackageSource.PYTHON, actual_package_name_macos
                except (subprocess.CalledProcessError, FileNotFoundError):
                    pass  # Continue to PyPI fallback

                # Fallback: Extract the actual PyPI package name (remove python3- prefix)
                pypi_package_name = package_name[
                    8:]  # Remove "python3-" prefix

                # Check if the package exists on PyPI
                if self._check_pypi_package_exists(pypi_package_name):
                    return PackageSource.PYTHON, pypi_package_name
                else:
                    print(
                        f"WARNING: {pypi_package_name} not found on PyPI, treating as system package."
                    )

            # For system packages, resolve with RHEL 8 for manylinux compatibility
            try:
                result_rhel = subprocess.run(
                    [
                        "rosdep",
                        "resolve",
                        "--os=rhel:8",
                        package_name,
                    ],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                lines_rhel = result_rhel.stdout.strip().split("\n")
                if len(lines_rhel) >= 2 and lines_rhel[0].startswith("#"):
                    actual_package_name_rhel = lines_rhel[1].strip()
                    return PackageSource.SYSTEM, actual_package_name_rhel
                else:
                    # Fallback if output format is unexpected
                    return PackageSource.SYSTEM, package_name
            except (subprocess.CalledProcessError, FileNotFoundError):
                print(
                    f"WARNING: Could not resolve {package_name} with rosdep. Assuming system package."
                )
                return PackageSource.SYSTEM, package_name

    def get_build_depends(self, pkg_name: str) -> set[Dependency]:
        """
        Get the build dependencies of a package.
        """
        dep_names = set()
        for cat in ["buildtool", "buildtool_export", "build", "build_export"]:
            deps = self.dependency_walker.get_depends(pkg_name, cat)
            dep_names.update(deps)

        dependencies = set()
        for dep_name in dep_names:
            if dep_name in IGNORE_LIST:
                continue
            if dep_name in DEPENDENCY_MAP:
                dependencies.add(DEPENDENCY_MAP[dep_name])
                continue
            source, actual_dep_name = self._determine_package_source_and_name(
                dep_name)
            dependencies.add(
                Dependency(ros_name=dep_name,
                           source=source,
                           dep_name=actual_dep_name))
        return dependencies

    def get_run_depends(self, pkg_name: str) -> set[Dependency]:
        """
        Get the run dependencies of a package.
        """
        dep_names = set()
        for cat in ["run", "exec"]:
            deps = self.dependency_walker.get_depends(pkg_name, cat)
            dep_names.update(deps)

        dependencies = set()
        for dep_name in dep_names:
            if dep_name in IGNORE_LIST:
                continue
            if dep_name in DEPENDENCY_MAP:
                dependencies.add(DEPENDENCY_MAP[dep_name])
                continue
            source, actual_dep_name = self._determine_package_source_and_name(
                dep_name)
            dependencies.add(
                Dependency(ros_name=dep_name,
                           source=source,
                           dep_name=actual_dep_name))
        return dependencies

    def get_test_depends(self, pkg_name):
        """
        Get the test dependencies of a package.
        """
        dep_names = self.dependency_walker.get_depends(pkg_name, "test")

        dependencies = set()
        for dep_name in dep_names:
            if dep_name in IGNORE_LIST:
                continue
            if dep_name in DEPENDENCY_MAP:
                dependencies.add(DEPENDENCY_MAP[dep_name])
                continue
            source, actual_dep_name = self._determine_package_source_and_name(
                dep_name)
            dependencies.add(
                Dependency(ros_name=dep_name,
                           source=source,
                           dep_name=actual_dep_name))
        return dependencies

    def get_released_repo(self, pkg_name):
        """
        Get the released repository of a package.
        """
        if pkg_name not in self.distro_file.release_packages:
            return None, None
        pkg = self.distro_file.release_packages[pkg_name]
        repo = self.distro_file.repositories[
            pkg.repository_name].release_repository
        return repo.url, repo.version

    def get_release_package_xml(self, pkg_name):
        """
        Get the package.xml of a package.
        """
        return self.distro_file.get_release_package_xml(pkg_name)

    def get_version(self, pkg_name):
        """
        Get the version of a package.
        """
        pkg = self.distro_file.release_packages[pkg_name]
        repo = self.distro_file.repositories[
            pkg.repository_name].release_repository
        return repo.version.split("-")[0]
