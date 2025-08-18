import os
import shutil
import subprocess
import sys
from typing import Optional, List, Dict, Any

from ros_python_wheels.list_deps import get_deps, categorize_debian_package


class ROSPythonPackageBuilder:
    def __init__(self, ros_distro: str, output_dir: str):
        self.ros_distro = ros_distro
        self.python_version = self._get_system_python_version()
        self.ros_python_path = (
            f"/opt/ros/{ros_distro}/lib/python{self.python_version}/site-packages"
        )

        # Common setup.py classifiers
        self.setup_classifiers = [
            "Development Status :: 4 - Beta",
            "Intended Audience :: Developers",
            "License :: OSI Approved :: Apache Software License",
            "Programming Language :: Python :: 3",
            f"Programming Language :: Python :: {self.python_version}",
            "Topic :: Scientific/Engineering :: Artificial Intelligence",
            "Topic :: Software Development :: Libraries :: Python Modules",
        ]
        self.output_dir = output_dir

    def _get_system_python_version(self) -> str:
        """Get the system Python version from /usr/bin/python3."""
        result = subprocess.run(
            [
                "/usr/bin/python3",
                "-c",
                "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')",
            ],
            capture_output=True,
            text=True,
            check=True,
        )
        version = result.stdout.strip()
        print(f"Detected system Python version: {version}")
        return version

    def _create_build_directory(self, ros_package: str, build_type: str = "") -> str:
        """Create and return a build directory path for the package."""
        if build_type:
            build_dir = os.path.join(self.output_dir, f"{ros_package}_{build_type}")
        else:
            build_dir = os.path.join(self.output_dir, ros_package)

        # Remove existing build directory if it exists
        if os.path.exists(build_dir):
            shutil.rmtree(build_dir)

        # Create the build directory
        os.makedirs(build_dir, exist_ok=True)
        return os.path.abspath(build_dir)

    def resolve_debian_package(self, ros_package: str) -> Optional[str]:
        """Use rosdep to resolve ROS package name to debian package name."""
        try:
            result = subprocess.run(
                ["rosdep", "resolve", ros_package],
                capture_output=True,
                text=True,
                check=True,
            )
            # rosdep resolve returns lines like: "#apt\nros-jazzy-unique-identifier-msgs"
            lines = result.stdout.strip().split("\n")
            for line in lines:
                if line.startswith("ros-"):
                    return line.strip()
            return None
        except subprocess.CalledProcessError as e:
            print(f"Error resolving package {ros_package}: {e}")
            return None

    def get_package_files(self, debian_package: str) -> List[str]:
        """Get list of files in debian package using dpkg -L."""
        try:
            result = subprocess.run(
                ["dpkg", "-L", debian_package],
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout.strip().split("\n")
        except subprocess.CalledProcessError as e:
            print(f"Error getting files for {debian_package}: {e}")
            return []

    def find_python_package_path(
        self, files: List[str], package_name: str
    ) -> Optional[str]:
        """Find the Python package path in the file list."""
        python_site_packages = f"{self.ros_python_path}/"

        for file_path in files:
            if file_path.startswith(python_site_packages):
                # Check if this is the package directory
                relative_path = file_path[len(python_site_packages) :]
                if (
                    relative_path.startswith(package_name + "/")
                    or relative_path == package_name
                ):
                    return f"{python_site_packages}{package_name}"

        return None

    def find_egg_info_path(self, files: List[str], package_name: str) -> Optional[str]:
        """Find the .egg-info directory for the package."""
        python_site_packages = f"{self.ros_python_path}/"

        for file_path in files:
            if file_path.startswith(python_site_packages):
                relative_path = file_path[len(python_site_packages) :]
                if relative_path.startswith(
                    f"{package_name}-"
                ) and relative_path.endswith(".egg-info"):
                    return file_path
        return None

    def extract_package_info(
        self, package_path: str, egg_info_path: Optional[str] = None
    ) -> Dict[str, Any]:
        """Extract package information from __init__.py, setup metadata, or egg-info."""
        package_basename = os.path.basename(package_path)
        info = {
            "name": f"ros-{package_basename.replace('_', '-')}",  # Add ros- prefix and convert underscores to dashes
            "version": "0.0.0",
            "description": "",
            "author": "",
            "author_email": "",
            "license": "",
            "url": "",
            "dependencies": [],
        }

        # Try to extract info from existing .egg-info directory first
        if egg_info_path and os.path.exists(egg_info_path):
            try:
                pkg_info_path = os.path.join(egg_info_path, "PKG-INFO")
                if os.path.exists(pkg_info_path):
                    with open(pkg_info_path, "r") as f:
                        content = f.read()

                    # Parse PKG-INFO format
                    for line in content.split("\n"):
                        if line.startswith("Name: "):
                            original_name = line[6:].strip()
                            # Ensure ros- prefix is maintained
                            if not original_name.startswith("ros-"):
                                info["name"] = f"ros-{original_name.replace('_', '-')}"
                            else:
                                info["name"] = original_name
                        elif line.startswith("Version: "):
                            info["version"] = line[9:].strip()
                        elif line.startswith("Summary: "):
                            info["description"] = line[9:].strip()
                        elif line.startswith("Author: "):
                            info["author"] = line[8:].strip()
                        elif line.startswith("Author-email: "):
                            info["author_email"] = line[14:].strip()

                print(f"Extracted info from egg-info: {info}")
            except Exception as e:
                print(f"Warning: Could not parse egg-info {egg_info_path}: {e}")

        # Try to find version in __init__.py
        init_file = os.path.join(package_path, "__init__.py")
        if os.path.exists(init_file):
            try:
                with open(init_file, "r") as f:
                    content = f.read()
                    # Look for common version patterns
                    import re

                    version_match = re.search(
                        r"__version__\s*=\s*['\"]([^'\"]+)['\"]", content
                    )
                    if version_match:
                        info["version"] = version_match.group(1)
            except Exception as e:
                print(f"Warning: Could not read {init_file}: {e}")

        # Try to find package.xml for ROS-specific metadata
        package_xml = None
        current_path = package_path
        while current_path != "/" and current_path:
            potential_xml = os.path.join(
                current_path,
                "..",
                "..",
                "..",
                "share",
                str(info["name"]),
                "package.xml",
            )
            if os.path.exists(potential_xml):
                package_xml = potential_xml
                break
            current_path = os.path.dirname(current_path)

        if package_xml:
            try:
                import xml.etree.ElementTree as ET

                tree = ET.parse(package_xml)
                root = tree.getroot()

                version_elem = root.find("version")
                if version_elem is not None and version_elem.text:
                    info["version"] = version_elem.text.strip()

                desc_elem = root.find("description")
                if desc_elem is not None and desc_elem.text:
                    # Clean up description text - replace newlines and extra whitespace
                    info["description"] = " ".join(desc_elem.text.strip().split())

                # Get license info
                license_elem = root.find("license")
                if license_elem is not None and license_elem.text:
                    info["license"] = license_elem.text.strip()
                else:
                    info["license"] = ""

                # Get maintainer info (primary author)
                maintainers = root.findall("maintainer")
                authors = root.findall("author")

                # Combine maintainers and authors, preferring maintainers
                all_people = []
                all_emails = []

                # First add maintainers
                for maintainer in maintainers:
                    if maintainer.text:
                        all_people.append(maintainer.text.strip())
                    email = maintainer.get("email", "")
                    if email:
                        all_emails.append(email)

                # Then add authors if we don't have maintainers
                if not all_people:
                    for author in authors:
                        if author.text:
                            all_people.append(author.text.strip())
                        email = author.get("email", "")
                        if email:
                            all_emails.append(email)

                # Set author and author_email
                if all_people:
                    info["author"] = ", ".join(all_people)
                if all_emails:
                    info["author_email"] = ", ".join(all_emails)

                # Generate homepage URL (common pattern for ROS packages)
                info["url"] = f"https://github.com/ros2/{package_basename}"

            except Exception as e:
                print(f"Warning: Could not parse package.xml {package_xml}: {e}")
        else:
            # If no package.xml found, try the standard location for the ROS package
            ros_package_name = package_basename
            self._parse_package_xml(ros_package_name, info)
            if not info.get("url"):
                info["url"] = f"https://github.com/ros2/{package_basename}"

        return info

    def gather_dependencies(self, ros_package: str) -> Dict[str, List[str]]:
        """Gather Python and ROS Python dependencies using get_deps."""
        ros_python_deps: set[str] = set()
        ros_runtime_deps: set[str] = set()
        ros_vendor_deps: set[str] = set()
        ros_linker_deps: set[str] = set()
        python_deps: set[str] = set()
        system_deps: set[str] = set()
        processed: set[str] = set()

        print(f"Gathering dependencies for {ros_package}...")

        get_deps(
            ros_package,
            ros_python_deps,
            ros_runtime_deps,
            ros_vendor_deps,
            ros_linker_deps,
            python_deps,
            system_deps,
            processed,
            ros_distro=self.ros_distro,
            level=0,
            recurse=False,
        )

        # if there are vendor dependencies, get their underlying system dependencies
        for vendor_dep in ros_vendor_deps:
            print(
                f"Converting vendor dependency {vendor_dep} to system dependencies..."
            )
            get_deps(
                vendor_dep,
                set(),
                set(),
                set(),
                set(),
                set(),
                system_deps,
                processed,
                ros_distro=self.ros_distro,
                level=0,
                recurse=False,
            )

        # Convert sets to sorted lists
        dependencies = {
            "python": sorted(python_deps),
            "ros_python": sorted(ros_python_deps),
            "ros_runtime": sorted(ros_runtime_deps),
            "ros_linker": sorted(ros_linker_deps),
            "system": sorted(system_deps),
        }

        # filter out cmake packages
        for dep_type, dep_list in dependencies.items():
            dependencies[dep_type] = [dep for dep in dep_list if "cmake" not in dep]

        print("Found dependencies:")
        for dep_type, dep_list in dependencies.items():
            print(f"  {dep_type}: {len(dep_list)} packages: {dep_list}")

        return dependencies

    def is_ros_python_package(self, ros_package: str) -> bool:
        """Check if it's a ROS python package (but not a system python package)."""
        if ros_package.startswith("python3-"):
            return False

        debian_package = self.resolve_debian_package(ros_package)
        if not debian_package:
            return False

        files = self.get_package_files(debian_package)
        python_package_path = self.find_python_package_path(files, ros_package)
        return python_package_path is not None

    def get_build_order(
        self,
        ros_package: str,
        visited: Optional[set[str]] = None,
        order: Optional[list[tuple[str, str]]] = None,
    ) -> list[tuple[str, str]]:
        """Determine the build order for Python packages, runtime libraries, and linker packages.

        Returns a list of tuples where each tuple is (package_name, package_type).
        package_type is one of 'python', 'runtime', or 'linker'.
        """
        if visited is None:
            visited = set()
        if order is None:
            order = []

        if ros_package in visited:
            return order

        visited.add(ros_package)

        debian_package = self.resolve_debian_package(ros_package)
        if not debian_package:
            print(f"Could not resolve debian package for {ros_package}")
            return order

        package_type = categorize_debian_package(debian_package, self.ros_distro)
        if (
            package_type in ("ros-python", "ros-runtime", "ros-linker")
            and "cmake" not in ros_package
        ):
            # Get dependencies for this package
            ros_python_deps: set[str] = set()
            ros_runtime_deps: set[str] = set()
            ros_vendor_deps: set[str] = set()
            ros_linker_deps: set[str] = set()
            python_deps: set[str] = set()
            system_deps: set[str] = set()
            processed: set[str] = set()

            get_deps(
                ros_package,
                ros_python_deps,
                ros_runtime_deps,
                ros_vendor_deps,
                ros_linker_deps,
                python_deps,
                system_deps,
                processed,
                ros_distro=self.ros_distro,
                level=0,
                recurse=True,
            )

            # Recursively get build order for each dependency
            for dep in ros_python_deps:
                self.get_build_order(dep, visited, order)

            for dep in ros_runtime_deps:
                self.get_build_order(dep, visited, order)

            for dep in ros_linker_deps:
                self.get_build_order(dep, visited, order)

            # Add current package to build order
            if package_type == "ros-python" and (ros_package, "python") not in order:
                order.append((ros_package, "python"))
            elif package_type == "ros-linker" and (ros_package, "linker") not in order:
                order.append((ros_package, "linker"))
            elif (
                package_type == "ros-runtime" and (ros_package, "runtime") not in order
            ):
                order.append((ros_package, "runtime"))
        else:
            print(f"Skipping package: {ros_package} ({package_type})")

        return order

    def get_ros_runtime_shared_libraries(
        self, ros_runtime_deps: List[str]
    ) -> Dict[str, List[str]]:
        """Get shared library files for ROS runtime dependencies."""
        shared_libs = {}
        ros_lib_path = f"/opt/ros/{self.ros_distro}/lib"

        print(
            f"Looking for shared libraries for {len(ros_runtime_deps)} ROS runtime dependencies..."
        )

        for dep in ros_runtime_deps:
            print(f"  Processing dependency: {dep}")

            # First, resolve the debian package name for this ROS dependency
            debian_package = self.resolve_debian_package(dep)
            if not debian_package:
                print(f"    Could not resolve debian package for {dep}")
                continue

            print(f"    Resolved to debian package: {debian_package}")

            # Get files from the debian package
            files = self.get_package_files(debian_package)
            if not files:
                print(f"    Could not get files for {debian_package}")
                continue

            # Find .so files in /opt/ros/<distro>/lib
            lib_files = []
            for file_path in files:
                if file_path.startswith(ros_lib_path) and (
                    file_path.endswith(".so") or ".so." in file_path
                ):
                    if os.path.exists(file_path):
                        lib_files.append(file_path)

            if lib_files:
                shared_libs[dep] = lib_files
                print(f"    Found {len(lib_files)} shared libraries:")
                for lib in lib_files:
                    print(f"      {lib}")
            else:
                print(f"    No shared libraries found for {dep}")

        return shared_libs

    def build_runtime_library_artifacts(self, ros_package: str) -> bool:
        """Build artifacts for a ROS runtime library package containing only shared libraries."""
        print(f"Building runtime library artifacts for ROS package: {ros_package}")

        # Step 1: Resolve debian package name
        debian_package = self.resolve_debian_package(ros_package)
        if not debian_package:
            print(f"Could not resolve debian package for {ros_package}")
            return False

        print(f"Resolved to debian package: {debian_package}")

        # Step 2: Get shared libraries for this package
        shared_libs = self.get_ros_runtime_shared_libraries([ros_package])
        if not shared_libs or ros_package not in shared_libs:
            print(f"No shared libraries found for {ros_package}")
            return False

        lib_files = shared_libs[ros_package]
        print(f"Found {len(lib_files)} shared libraries for {ros_package}")

        # Step 3: Create package info for runtime library
        package_info = self._create_default_package_info(
            ros_package, "runtime libraries"
        )

        # Step 3.5: Gather dependencies for runtime library
        dependencies = self.gather_dependencies(ros_package)
        package_info["dependencies"] = dependencies

        # Try to get version and description from package.xml if available
        self._parse_package_xml(ros_package, package_info, "runtime libraries")

        # Step 4: Create build directory
        build_dir = self._create_build_directory(ros_package, "runtime")
        print(f"Using build directory: {build_dir}")

        # Create package directory structure
        self._create_temp_package_directory(
            build_dir, ros_package, f"ROS {ros_package} runtime libraries package."
        )

        # Create setup.py for runtime library package
        setup_py_path = self.create_runtime_setup_py(
            build_dir, package_info, ros_package
        )
        print(f"Created setup.py at: {setup_py_path}")

        # Create MANIFEST.in
        manifest_path = self.create_manifest_in(
            build_dir, f"ros_{ros_package.replace('-', '_')}"
        )
        print(f"Created MANIFEST.in at: {manifest_path}")
        return True

    def _write_setup_py(self, build_dir: str, setup_py_content: str) -> str:
        """Helper method to write setup.py content to file."""
        setup_py_path = os.path.join(build_dir, "setup.py")
        with open(setup_py_path, "w") as f:
            f.write(setup_py_content)
        return setup_py_path

    def _get_classifiers_string(self) -> str:
        """Helper method to get formatted classifiers string for setup.py."""
        return ",\n        ".join(
            f'"{classifier}"' for classifier in self.setup_classifiers
        )

    def _parse_package_xml(
        self,
        ros_package: str,
        package_info: Dict[str, Any],
        description_prefix: str = "",
    ) -> None:
        """Parse package.xml to extract version and description."""
        try:
            package_xml = f"/opt/ros/{self.ros_distro}/share/{ros_package}/package.xml"
            if os.path.exists(package_xml):
                import xml.etree.ElementTree as ET

                tree = ET.parse(package_xml)
                root = tree.getroot()

                version_elem = root.find("version")
                if version_elem is not None and version_elem.text:
                    package_info["version"] = version_elem.text.strip()

                desc_elem = root.find("description")
                if desc_elem is not None and desc_elem.text:
                    # Clean up description text - replace newlines and extra whitespace
                    clean_desc = " ".join(desc_elem.text.strip().split())
                    if description_prefix:
                        package_info["description"] = (
                            f"ROS {ros_package} {description_prefix} - {clean_desc}"
                        )
                    else:
                        package_info["description"] = clean_desc

                # Get license info
                license_elem = root.find("license")
                if license_elem is not None and license_elem.text:
                    package_info["license"] = license_elem.text.strip()

                # Get maintainer info (primary author)
                maintainers = root.findall("maintainer")
                authors = root.findall("author")

                # Combine maintainers and authors, preferring maintainers
                all_people = []
                all_emails = []

                # First add maintainers
                for maintainer in maintainers:
                    if maintainer.text:
                        all_people.append(maintainer.text.strip())
                    email = maintainer.get("email", "")
                    if email:
                        all_emails.append(email)

                # Then add authors if we don't have maintainers
                if not all_people:
                    for author in authors:
                        if author.text:
                            all_people.append(author.text.strip())
                        email = author.get("email", "")
                        if email:
                            all_emails.append(email)

                # Set author and author_email
                if all_people:
                    package_info["author"] = ", ".join(all_people)
                if all_emails:
                    package_info["author_email"] = ", ".join(all_emails)

        except Exception as e:
            print(f"Warning: Could not parse package.xml: {e}")

    def _prepare_install_requires(
        self, package_info: Dict[str, Any]
    ) -> tuple[str, str]:
        """Prepare install_requires list from dependencies and format for setup.py.

        Returns:
            tuple: (install_requires_str, extras_require_str)
        """
        install_requires = []
        extras_require = {}

        # Add Python dependencies
        if "dependencies" in package_info and "python" in package_info["dependencies"]:
            for dep in package_info["dependencies"]["python"]:
                # Convert debian package names to pip package names where possible
                if dep.startswith("python3-"):
                    pip_name = dep[8:].replace(
                        "-", "_"
                    )  # Remove python3- prefix and convert dashes
                    # special case for pyyaml
                    if pip_name == "yaml":
                        pip_name = "pyyaml"
                    install_requires.append(pip_name)
                else:
                    install_requires.append(dep)

        # Add ROS Python dependencies as pip installable packages with ros- prefix
        if (
            "dependencies" in package_info
            and "ros_python" in package_info["dependencies"]
        ):
            for dep in package_info["dependencies"]["ros_python"]:
                # Convert ROS package names to pip package names with ros- prefix
                pip_name = f"ros-{dep.replace('_', '-')}"
                install_requires.append(pip_name)

        # Add ROS runtime dependencies as pip installable runtime packages with ros- prefix
        if (
            "dependencies" in package_info
            and "ros_runtime" in package_info["dependencies"]
        ):
            for dep in package_info["dependencies"]["ros_runtime"]:
                # Convert ROS runtime package names to pip package names with ros- prefix
                pip_name = f"ros-{dep.replace('_', '-')}"
                install_requires.append(pip_name)

        # Add ROS other dependencies (linker packages) as pip installable packages with ros- prefix
        if (
            "dependencies" in package_info
            and "ros_linker" in package_info["dependencies"]
        ):
            for dep in package_info["dependencies"]["ros_linker"]:
                # Convert ROS other package names to pip package names with ros- prefix
                pip_name = f"ros-{dep.replace('_', '-')}"
                if "vendor" not in pip_name:  # Skip vendor packages
                    install_requires.append(pip_name)

        # Special case for ros-rclpy: add fastrtps extra
        if package_info.get("name") == "ros-rclpy":
            extras_require["fastrtps"] = ["ros-rmw-fastrtps-cpp"]

        # Format install_requires for the setup.py
        if install_requires:
            install_requires_str = ",\n        ".join(
                f'"{dep}"' for dep in install_requires
            )
            install_requires_formatted = f"[\n        {install_requires_str},\n    ]"
        else:
            install_requires_formatted = "[]"

        # Format extras_require for the setup.py
        if extras_require:
            extras_parts = []
            for extra, deps in extras_require.items():
                deps_str = ", ".join(f'"{dep}"' for dep in deps)
                extras_parts.append(f'"{extra}": [{deps_str}]')
            extras_formatted = (
                "{\n        " + ",\n        ".join(extras_parts) + ",\n    }"
            )
        else:
            extras_formatted = "{}"

        return install_requires_formatted, extras_formatted

    def _create_default_package_info(
        self, ros_package: str, description_suffix: str = ""
    ) -> Dict[str, Any]:
        """Create default package info structure."""
        base_description = f"ROS {ros_package}"
        if description_suffix:
            base_description += f" {description_suffix}"

        return {
            "name": f"ros-{ros_package.replace('_', '-')}",
            "version": "0.0.0",
            "description": base_description,
            "author": "ROS Community",
            "author_email": "",
            "license": "Apache License 2.0",
            "url": f"https://github.com/ros2/{ros_package}",
            "dependencies": {},
        }

    def _create_temp_package_directory(
        self, build_dir: str, ros_package: str, init_docstring: str
    ) -> str:
        """Create package directory structure in temp directory."""
        package_name = f"ros_{ros_package.replace('-', '_')}"
        package_dest = os.path.join(build_dir, package_name)
        os.makedirs(package_dest, exist_ok=True)

        # Create empty __init__.py to make it a Python package
        init_file = os.path.join(package_dest, "__init__.py")
        with open(init_file, "w") as f:
            f.write(f'"""{init_docstring}"""\n')

        return package_dest

    def _generate_setup_py_content(
        self,
        package_info: Dict[str, Any],
        install_requires_section: str,
        packages_spec: str = "find_packages()",
        package_data_spec: str = "package_data",
        additional_setup_code: str = "",
        has_ext_modules: bool = True,
        extras_require_section: str = "{}",
    ) -> str:
        """Generate setup.py content with common structure."""
        setup_content = f'''#!/usr/bin/env python3

from setuptools import setup, find_packages
from setuptools.extension import Extension
import glob
import os

{additional_setup_code}

setup(
    name="{package_info["name"]}",
    version="{package_info["version"]}",
    description="""{package_info["description"].replace('"', '\\"')}""",
    long_description="""{package_info["description"].replace('"', '\\"')}""",
    author="{package_info["author"]}",
    author_email="{package_info["author_email"]}",
    url="{package_info.get("url", "")}",
    license="{package_info.get("license", "Apache License 2.0")}",
    packages={packages_spec},
    package_data={package_data_spec},
    include_package_data=True,
    install_requires={install_requires_section},
    extras_require={extras_require_section},
    classifiers=[
        {self._get_classifiers_string()},
    ],
    python_requires=">=3.8",
    zip_safe=False,
    has_ext_modules=lambda: {str(has_ext_modules)},
)
'''
        return setup_content

    def create_runtime_setup_py(self, build_dir, package_info, ros_package_name) -> str:
        target_pkg_dir = os.path.join(build_dir, "ros_runtime_libs")
        os.makedirs(target_pkg_dir, exist_ok=True)

        # Make ros_runtime_libs a proper package
        open(os.path.join(target_pkg_dir, "__init__.py"), "a").close()

        # Get .so files from debian package and copy them into ros_runtime_libs/
        # Get the debian package name for this ROS package
        debian_package = self.resolve_debian_package(ros_package_name)
        if debian_package:
            # Get all files from the debian package
            debian_files = self.get_package_files(debian_package)

            for file_path in debian_files:
                if file_path.endswith((".so", ".so.*")) or ".so." in file_path:
                    if os.path.exists(file_path):
                        file_name = os.path.basename(file_path)
                        dest_path = os.path.join(target_pkg_dir, file_name)
                        try:
                            # Copy the .so file to ros_runtime_libs directory
                            shutil.copy2(file_path, dest_path)
                            print(f"  Copied {file_name} to ros_runtime_libs/")
                        except Exception as e:
                            print(f"  Warning: Could not copy {file_name}: {e}")
        else:
            print(f"  Warning: Could not resolve debian package for {ros_package_name}")

        # Clean up empty dirs in original package (if they exist)
        src_pkg_dir = os.path.join(build_dir, ros_package_name)
        if os.path.exists(src_pkg_dir):
            for root, dirs, files in os.walk(src_pkg_dir, topdown=False):
                if not files and not dirs:
                    try:
                        os.rmdir(root)
                    except OSError:
                        pass  # Directory not empty or doesn't exist

        # Prepare install_requires list from dependencies
        install_requires_section, extras_require_section = (
            self._prepare_install_requires(package_info)
        )

        setup_py_content = self._generate_setup_py_content(
            package_info=package_info,
            install_requires_section=install_requires_section,
            packages_spec='find_packages(include=["ros_runtime_libs"])',
            package_data_spec='{"ros_runtime_libs": ["*.so", "*.so.*"]}',
            additional_setup_code="",
            extras_require_section=extras_require_section,
        )
        return self._write_setup_py(build_dir, setup_py_content)

    def create_setup_py(
        self,
        build_dir: str,
        package_info: Dict[str, Any],
        ros_package_name: str,
    ) -> str:
        """Create a setup.py file for the package."""
        # Prepare install_requires list from dependencies
        install_requires_section, extras_require_section = (
            self._prepare_install_requires(package_info)
        )

        # Create ros_runtime_libs directory and symlink .so files
        ros_runtime_libs_dir = os.path.join(build_dir, "ros_runtime_libs")
        os.makedirs(ros_runtime_libs_dir, exist_ok=True)

        # Make ros_runtime_libs a proper package
        with open(os.path.join(ros_runtime_libs_dir, "__init__.py"), "w") as f:
            f.write('"""ROS runtime libraries symlinks."""\n')

        # Find .so files from debian package and create symlinks in ros_runtime_libs/
        # Get the debian package name for this ROS package
        debian_package = self.resolve_debian_package(ros_package_name)
        if debian_package:
            # Get all files from the debian package
            debian_files = self.get_package_files(debian_package)

            for file_path in debian_files:
                if file_path.endswith(".so") or ".so." in file_path:
                    if os.path.exists(file_path):
                        file_name = os.path.basename(file_path)
                        symlink_path = os.path.join(ros_runtime_libs_dir, file_name)
                        try:
                            # Create relative symlink to preserve package structure
                            rel_path = os.path.relpath(file_path, ros_runtime_libs_dir)
                            os.symlink(rel_path, symlink_path)
                            print(f"  Created symlink: {file_name} -> {rel_path}")
                        except Exception as e:
                            print(
                                f"  Warning: Could not create symlink for {file_name}: {e}"
                            )
        else:
            print(f"  Warning: Could not resolve debian package for {ros_package_name}")

        additional_setup_code = f'''# Find all shared libraries and C extension files
package_data = {{}}
ext_modules = []

# Include all .so files, .c files, and other binary extensions for the main package
for root, dirs, files in os.walk("{ros_package_name}"):
    for file in files:
        if file.endswith(('.so', '.c', '.pyx', '.pxd')):
            rel_path = os.path.relpath(os.path.join(root, file), "{ros_package_name}")
            if "{ros_package_name}" not in package_data:
                package_data["{ros_package_name}"] = []
            package_data["{ros_package_name}"].append(rel_path)

# Include symlinked .so files in ros_runtime_libs
ros_runtime_libs_files = []
if os.path.exists("ros_runtime_libs"):
    for file in os.listdir("ros_runtime_libs"):
        if file.endswith((".so", ".so.*")) or ".so." in file:
            ros_runtime_libs_files.append(file)
    if ros_runtime_libs_files:
        package_data["ros_runtime_libs"] = ros_runtime_libs_files'''

        setup_py_content = self._generate_setup_py_content(
            package_info=package_info,
            install_requires_section=install_requires_section,
            packages_spec="find_packages()",
            package_data_spec="package_data",
            additional_setup_code=additional_setup_code,
            extras_require_section=extras_require_section,
        )
        return self._write_setup_py(build_dir, setup_py_content)

    def create_manifest_in(self, build_dir: str, package_name: str) -> str:
        """Create a MANIFEST.in file to include all package files."""
        manifest_content = f"""# Include all files in the package
recursive-include {package_name} *
recursive-include ros_runtime_libs *
global-exclude *.pyc
global-exclude __pycache__
"""
        manifest_path = os.path.join(build_dir, "MANIFEST.in")
        with open(manifest_path, "w") as f:
            f.write(manifest_content)

        return manifest_path

    def build_linker_artifacts(self, ros_package: str) -> bool:
        """Build artifacts for a ROS other package as a linker package with dependencies only."""
        print(f"Building linker artifacts for ROS package: {ros_package}")

        # Step 1: Resolve debian package name
        debian_package = self.resolve_debian_package(ros_package)
        if not debian_package:
            print(f"Could not resolve debian package for {ros_package}")
            return False

        print(f"Resolved to debian package: {debian_package}")

        # Step 2: Extract package info
        package_info = self._create_default_package_info(
            ros_package, "linker package with dependencies"
        )

        # Try to get version and description from package.xml if available
        self._parse_package_xml(ros_package, package_info, "linker package")

        # Step 3: Gather dependencies
        dependencies = self.gather_dependencies(ros_package)
        package_info["dependencies"] = dependencies

        # Step 4: Create build directory
        build_dir = self._create_build_directory(ros_package, "linker")
        print(f"Using build directory: {build_dir}")

        # Create package directory structure
        self._create_temp_package_directory(
            build_dir,
            ros_package,
            f"ROS {ros_package} linker package with dependencies only.",
        )

        # Create setup.py for linker package
        setup_py_path = self.create_setup_py(build_dir, package_info, ros_package)
        print(f"Created setup.py at: {setup_py_path}")

        # Create MANIFEST.in
        manifest_path = self.create_manifest_in(build_dir, ros_package)
        print(f"Created MANIFEST.in at: {manifest_path}")
        return True

    def build_artifacts(self, ros_package: str) -> bool:
        """Build artifacts from a ROS package."""
        print(f"Building artifacts for ROS package: {ros_package}")

        # Step 1: Resolve debian package name
        debian_package = self.resolve_debian_package(ros_package)
        if not debian_package:
            print(f"Could not resolve debian package for {ros_package}")
            return False

        print(f"Resolved to debian package: {debian_package}")

        # Step 2: Get package files
        files = self.get_package_files(debian_package)
        if not files:
            print(f"Could not get files for {debian_package}")
            return False

        # Step 3: Find Python package path
        python_package_path = self.find_python_package_path(files, ros_package)
        if not python_package_path:
            print(
                f"Could not find Python package for {ros_package} in {self.ros_python_path}"
            )
            print("Available Python files:")
            python_files = [f for f in files if self.ros_python_path in f]
            for f in python_files[:10]:  # Show first 10
                print(f"  {f}")
            if len(python_files) > 10:
                print(f"  ... and {len(python_files) - 10} more")
            return False

        print(f"Found Python package at: {python_package_path}")

        # Step 3.5: Find egg-info directory
        egg_info_path = self.find_egg_info_path(files, ros_package)
        if egg_info_path:
            print(f"Found egg-info at: {egg_info_path}")
        else:
            print("No egg-info directory found")

        # Step 4: Extract package info
        package_info = self.extract_package_info(python_package_path, egg_info_path)
        print(f"Package info: {package_info}")

        # Step 4.5: Gather dependencies
        dependencies = self.gather_dependencies(ros_package)
        package_info["dependencies"] = dependencies

        # Step 5: Create build directory
        build_dir = self._create_build_directory(ros_package, "python")
        print(f"Using build directory: {build_dir}")

        # Copy the Python package to temp directory
        package_dest = os.path.join(build_dir, ros_package)
        try:
            shutil.copytree(python_package_path, package_dest)
            print(f"Copied package to: {package_dest}")
        except Exception as e:
            print(f"Error copying package: {e}")
            return False

        # Create setup.py
        setup_py_path = self.create_setup_py(build_dir, package_info, ros_package)
        print(f"Created setup.py at: {setup_py_path}")

        # Create MANIFEST.in
        manifest_path = self.create_manifest_in(build_dir, ros_package)
        print(f"Created MANIFEST.in at: {manifest_path}")
        return True

    def build_artifacts_recursive(self, ros_package: str) -> bool:
        """Build artifacts for a ROS package and all its dependencies (both Python and runtime)."""
        print(f"🚀 Starting recursive build for ROS package: {ros_package}")

        # Determine build order for both Python packages and runtime libraries
        print("\n🔍 Determining build order...")
        build_order = self.get_build_order(ros_package)

        if not build_order:
            print(f"❌ No packages found to build for {ros_package}")
            return False

        # Separate runtime, python, and linker packages for display
        runtime_packages = [(pkg, typ) for pkg, typ in build_order if typ == "runtime"]
        python_packages = [(pkg, typ) for pkg, typ in build_order if typ == "python"]
        linker_packages = [(pkg, typ) for pkg, typ in build_order if typ == "linker"]

        total_packages = len(build_order)
        print(f"📦 Build order determined ({total_packages} total packages):")
        if runtime_packages:
            print(f"  Runtime library packages ({len(runtime_packages)}):")
            for i, (pkg, _) in enumerate(runtime_packages, 1):
                print(f"    {i}. {pkg}")
        if python_packages:
            print(f"  Python packages ({len(python_packages)}):")
            for i, (pkg, _) in enumerate(python_packages, 1):
                print(f"    {i}. {pkg}")
        if linker_packages:
            print(f"  Linker packages ({len(linker_packages)}):")
            for i, (pkg, _) in enumerate(linker_packages, 1):
                print(f"    {i}. {pkg}")

        # Build packages in order
        built_packages: set[str] = set()

        for i, (pkg, pkg_type) in enumerate(build_order, 1):
            if pkg_type == "runtime":
                print(
                    f"\n🔨 Building runtime libraries for {pkg} ({i}/{total_packages})..."
                )
                success = self.build_runtime_library_artifacts(pkg)

                if success:
                    built_packages.add(pkg)
                    print(f"✅ Successfully built runtime artifacts for {pkg}")
                else:
                    print(f"❌ Failed to build runtime artifacts for {pkg}")
                    print(f"💥 Recursive build failed at runtime package {pkg}")
                    return False

            elif pkg_type == "python":
                print(f"\n🔨 Building Python package {pkg} ({i}/{total_packages})...")
                success = self.build_artifacts(pkg)

                if success:
                    built_packages.add(pkg)
                    print(f"✅ Successfully built {pkg}")
                else:
                    print(f"❌ Failed to build {pkg}")
                    print(f"💥 Recursive build failed at package {pkg}")
                    return False

            elif pkg_type == "linker":
                print(f"\n🔨 Building linker package {pkg} ({i}/{total_packages})...")
                success = self.build_linker_artifacts(pkg)

                if success:
                    built_packages.add(f"{pkg}-linker")
                    print(f"✅ Successfully built linker package {pkg}")
                else:
                    print(f"❌ Failed to build linker package {pkg}")
                    print(f"💥 Recursive build failed at linker package {pkg}")
                    return False

        print("\n🎉 Recursive build completed successfully!")
        print(f"📦 Built packages: {sorted(built_packages)}")
        return True


def build_python_artifacts(
    package_name: str,
    ros_distro: str = "jazzy",
    output_dir: str = "build",
    no_recurse: bool = False,
):
    """Build Python artifacts from ROS packages. The artifacts can then be used to build
    Python wheels.

    Args:
        package_name (str): The name of the ROS package to build artifacts for.
        ros_distro (str): The ROS distribution to target (default: "jazzy").
        output_dir (str): The directory to output built artifacts (default: "build").
        no_recurse (bool): If True, do not build dependencies recursively (default: False).
    """
    builder = ROSPythonPackageBuilder(ros_distro=ros_distro, output_dir=output_dir)

    if no_recurse:
        success = builder.build_artifacts(package_name)
    else:
        success = builder.build_artifacts_recursive(package_name)

    if success:
        print(f"Successfully built artifacts for {package_name}")
    else:
        print(f"Failed to build artifacts for {package_name}")
        sys.exit(1)
