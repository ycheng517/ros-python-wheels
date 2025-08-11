#!/usr/bin/env python3
"""
Script to convert ROS Python packages into Python wheels.

This script takes a ROS package name, resolves its debian package using rosdep,
extracts the Python package from /opt/ros/<ROS_DISTRO>/lib/python3.12/site-packages/,
and creates a wheel file.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from typing import Optional, List, Dict, Any

from ros_python_wheels.list_deps import get_deps


class ROSPythonPackageBuilder:
    def __init__(self, ros_distro: str = "jazzy"):
        self.ros_distro = ros_distro
        self.python_version = "3.12"  # Default for ROS Jazzy
        self.ros_python_path = (
            f"/opt/ros/{ros_distro}/lib/python{self.python_version}/site-packages"
        )

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
                    info["description"] = desc_elem.text.strip()

                # Get maintainer info
                maintainer_elem = root.find("maintainer")
                if maintainer_elem is not None:
                    info["author"] = (
                        maintainer_elem.text.strip() if maintainer_elem.text else ""
                    )
                    info["author_email"] = maintainer_elem.get("email", "")

            except Exception as e:
                print(f"Warning: Could not parse package.xml {package_xml}: {e}")

        return info

    def gather_dependencies(self, ros_package: str) -> Dict[str, List[str]]:
        """Gather Python and ROS Python dependencies using get_deps."""
        ros_python_deps: set[str] = set()
        ros_runtime_deps: set[str] = set()
        ros_other_deps: set[str] = set()
        python_deps: set[str] = set()
        system_deps: set[str] = set()
        processed: set[str] = set()

        print(f"Gathering dependencies for {ros_package}...")

        get_deps(
            ros_package,
            ros_python_deps,
            ros_runtime_deps,
            ros_other_deps,
            python_deps,
            system_deps,
            processed,
            level=0,
            recurse=False,
        )

        # Convert sets to sorted lists
        dependencies = {
            "python": sorted(python_deps),
            "ros_python": sorted(ros_python_deps),
            "ros_runtime": sorted(ros_runtime_deps),
            "ros_other": sorted(ros_other_deps),
            "system": sorted(system_deps),
        }

        print("Found dependencies:")
        print(f"  Python: {len(dependencies['python'])} packages")
        print(f"  ROS Python: {len(dependencies['ros_python'])} packages")
        print(f"  ROS Runtime: {len(dependencies['ros_runtime'])} packages")
        print(f"  ROS Other: {len(dependencies['ros_other'])} packages")
        print(f"  System: {len(dependencies['system'])} packages")

        return dependencies

    def get_all_ros_python_dependencies(self, ros_package: str) -> List[str]:
        """Get all ROS Python dependencies recursively."""
        ros_python_deps: set[str] = set()
        ros_runtime_deps: set[str] = set()
        ros_other_deps: set[str] = set()
        python_deps: set[str] = set()
        system_deps: set[str] = set()
        processed: set[str] = set()

        print(f"Gathering all ROS Python dependencies for {ros_package}...")

        get_deps(
            ros_package,
            ros_python_deps,
            ros_runtime_deps,
            ros_other_deps,
            python_deps,
            system_deps,
            processed,
            level=0,
            recurse=True,  # Enable recursion to get all dependencies
        )

        return sorted(ros_python_deps)

    def wheel_exists(self, package_name: str, output_dir: str) -> bool:
        """Check if a wheel file already exists for the given package."""
        if not os.path.exists(output_dir):
            return False

        # Convert package name to expected wheel prefix (ros- prefix with underscores)
        wheel_prefix = f"ros_{package_name.replace('-', '_')}"

        for filename in os.listdir(output_dir):
            if filename.startswith(wheel_prefix) and filename.endswith(".whl"):
                print(f"Found existing wheel for {package_name}: {filename}")
                return True

        return False

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
        order: Optional[list[str]] = None,
    ) -> list[str]:
        """Determine the build order for a package and its dependencies using topological sort."""
        if visited is None:
            visited = set()
        if order is None:
            order = []

        if ros_package in visited:
            return order

        visited.add(ros_package)

        # Only add to build order if it's a Python package
        if self.is_ros_python_package(ros_package):
            # Get direct ROS Python dependencies for this package
            ros_python_deps: set[str] = set()
            ros_runtime_deps: set[str] = set()
            ros_other_deps: set[str] = set()
            python_deps: set[str] = set()
            system_deps: set[str] = set()
            processed: set[str] = set()

            get_deps(
                ros_package,
                ros_python_deps,
                ros_runtime_deps,
                ros_other_deps,
                python_deps,
                system_deps,
                processed,
                level=0,
                recurse=False,  # Only get direct dependencies for ordering
            )

            # Recursively get build order for each ROS Python dependency
            for dep in ros_python_deps:
                self.get_build_order(dep, visited, order)

            # Add current package to order if not already present
            if ros_package not in order:
                order.append(ros_package)

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

    def copy_shared_libraries(
        self, temp_dir: str, package_name: str, shared_libs: Dict[str, List[str]]
    ) -> None:
        """Copy shared libraries to the package directory."""
        if not shared_libs:
            return

        package_dir = os.path.join(temp_dir, package_name)
        libs_dir = os.path.join(package_dir, "lib")

        print(f"Copying shared libraries to {libs_dir}...")
        os.makedirs(libs_dir, exist_ok=True)

        for dep, lib_files in shared_libs.items():
            print(f"  Copying libraries for {dep}:")
            for lib_file in lib_files:
                if os.path.exists(lib_file):
                    lib_name = os.path.basename(lib_file)
                    dest_path = os.path.join(libs_dir, lib_name)
                    try:
                        shutil.copy2(lib_file, dest_path)
                        print(f"    Copied {lib_name}")
                    except Exception as e:
                        print(f"    Error copying {lib_file}: {e}")

    def create_setup_py(
        self, temp_dir: str, package_info: Dict[str, Any], package_name: str
    ) -> str:
        """Create a setup.py file for the package."""

        # Prepare install_requires list from dependencies
        install_requires = []

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

        # Format install_requires for the setup.py
        if install_requires:
            install_requires_str = ",\n        ".join(
                f'"{dep}"' for dep in install_requires
            )
            install_requires_section = f"[\n        {install_requires_str},\n    ]"
        else:
            install_requires_section = "[]"

        setup_py_content = f'''#!/usr/bin/env python3

from setuptools import setup, find_packages
from setuptools.extension import Extension
import glob
import os

# Find all shared libraries and C extension files
package_data = {{}}
ext_modules = []

# Include all .so files, .c files, and other binary extensions
for root, dirs, files in os.walk("{package_name}"):
    for file in files:
        if file.endswith(('.so', '.c', '.pyx', '.pxd')):
            rel_path = os.path.relpath(os.path.join(root, file), "{package_name}")
            if "{package_name}" not in package_data:
                package_data["{package_name}"] = []
            package_data["{package_name}"].append(rel_path)

setup(
    name="{package_info["name"]}",
    version="{package_info["version"]}",
    description="{package_info["description"]}",
    author="{package_info["author"]}",
    author_email="{package_info["author_email"]}",
    packages=find_packages(),
    package_data=package_data,
    include_package_data=True,
    install_requires={install_requires_section},
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.12",
        "Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    python_requires=">=3.8",
    zip_safe=False,  # Important for packages with shared libraries
    has_ext_modules=lambda: True,  # Mark as having extensions for platform-specific wheels
)
'''
        setup_py_path = os.path.join(temp_dir, "setup.py")
        with open(setup_py_path, "w") as f:
            f.write(setup_py_content)

        return setup_py_path

    def create_manifest_in(self, temp_dir: str, package_name: str) -> str:
        """Create a MANIFEST.in file to include all binary files."""
        manifest_content = f"""# Include all files in the package
recursive-include {package_name} *
global-exclude *.pyc
global-exclude __pycache__
"""
        manifest_path = os.path.join(temp_dir, "MANIFEST.in")
        with open(manifest_path, "w") as f:
            f.write(manifest_content)

        return manifest_path

    def build_wheel(self, ros_package: str, output_dir: str = "dist") -> bool:
        """Build a Python wheel from a ROS package."""
        print(f"Building wheel for ROS package: {ros_package}")

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

        # Step 5: Create temporary build directory
        with tempfile.TemporaryDirectory(delete=False) as temp_dir:
            print(f"Using temporary directory: {temp_dir}")

            # Copy the Python package to temp directory
            package_dest = os.path.join(temp_dir, ros_package)
            try:
                shutil.copytree(python_package_path, package_dest)
                print(f"Copied package to: {package_dest}")
            except Exception as e:
                print(f"Error copying package: {e}")
                return False

            # Step 5.5: Handle ROS runtime dependencies - copy shared libraries
            if dependencies.get("ros_runtime"):
                shared_libs = self.get_ros_runtime_shared_libraries(
                    dependencies["ros_runtime"]
                )
                if shared_libs:
                    self.copy_shared_libraries(temp_dir, ros_package, shared_libs)
                else:
                    print("No shared libraries found for ROS runtime dependencies")

            # Create setup.py
            setup_py_path = self.create_setup_py(temp_dir, package_info, ros_package)
            print(f"Created setup.py at: {setup_py_path}")

            # Create MANIFEST.in
            manifest_path = self.create_manifest_in(temp_dir, ros_package)
            print(f"Created MANIFEST.in at: {manifest_path}")

            # Create output directory
            os.makedirs(output_dir, exist_ok=True)

            # Build the wheel
            try:
                cmd = [
                    sys.executable,
                    "-m",
                    "pip",
                    "wheel",
                    "--no-build-isolation",
                    "--wheel-dir",
                    os.path.abspath(output_dir),
                    "--find-links",  # TODO (Yifei): may need to adjust this
                    os.path.abspath(output_dir),
                    temp_dir,
                ]
                print(f"Running: {' '.join(cmd)}")

                result = subprocess.run(cmd, check=True, capture_output=True, text=True)
                print("Wheel built successfully!")
                print(f"Output: {result.stdout}")
                if result.stderr:
                    print(f"Warnings: {result.stderr}")

                return True

            except subprocess.CalledProcessError as e:
                print(f"Error building wheel: {e}")
                print(f"stdout: {e.stdout}")
                print(f"stderr: {e.stderr}")
                return False

    def build_wheel_recursive(self, ros_package: str, output_dir: str = "dist") -> bool:
        """Build a Python wheel for a ROS package and all its ROS Python dependencies."""
        print(f"🚀 Starting recursive build for ROS package: {ros_package}")
        print(f"📁 Output directory: {output_dir}")

        # Create output directory
        os.makedirs(output_dir, exist_ok=True)

        # Determine build order
        print("\n🔍 Determining build order...")
        build_order = self.get_build_order(ros_package)

        if not build_order:
            print(f"❌ No Python packages found to build for {ros_package}")
            return False

        print(f"📦 Build order determined ({len(build_order)} packages):")
        for i, pkg in enumerate(build_order, 1):
            print(f"  {i}. {pkg}")

        # Build packages in order
        built_packages: set[str] = set()

        for pkg in build_order:
            # Check if wheel already exists
            if self.wheel_exists(pkg, output_dir):
                print(f"\n✅ Wheel for {pkg} already exists, skipping build")
                built_packages.add(pkg)
                continue

            print(
                f"\n🔨 Building {pkg} ({len(built_packages) + 1}/{len(build_order)})..."
            )
            success = self.build_wheel(pkg, output_dir)

            if success:
                built_packages.add(pkg)
                print(f"✅ Successfully built {pkg}")
            else:
                print(f"❌ Failed to build {pkg}")
                print(f"💥 Recursive build failed at package {pkg}")
                return False

        print("\n🎉 Recursive build completed successfully!")
        print(f"📦 Built packages: {sorted(built_packages)}")
        return True


def main():
    parser = argparse.ArgumentParser(
        description="Convert ROS Python packages to Python wheels"
    )
    parser.add_argument(
        "package", help="ROS package name (e.g., unique_identifier_msgs)"
    )
    parser.add_argument(
        "--ros-distro", default="jazzy", help="ROS distribution (default: jazzy)"
    )
    parser.add_argument(
        "--output-dir",
        default="dist",
        help="Output directory for wheels (default: dist)",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument(
        "--recursive",
        "-r",
        action="store_true",
        help="Build all ROS Python dependencies recursively before building the target package",
    )

    args = parser.parse_args()

    if args.verbose:
        print(f"Building wheel for: {args.package}")
        print(f"ROS distro: {args.ros_distro}")
        print(f"Output directory: {args.output_dir}")
        print(f"Recursive mode: {args.recursive}")

    builder = ROSPythonPackageBuilder(ros_distro=args.ros_distro)

    if args.recursive:
        success = builder.build_wheel_recursive(args.package, args.output_dir)
    else:
        success = builder.build_wheel(args.package, args.output_dir)

    if success:
        print(f"Successfully built wheel for {args.package}")
        sys.exit(0)
    else:
        print(f"Failed to build wheel for {args.package}")
        sys.exit(1)


if __name__ == "__main__":
    main()
