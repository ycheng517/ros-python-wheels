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
        info = {
            "name": os.path.basename(package_path),
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
                            info["name"] = line[6:].strip()
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

    def create_setup_py(
        self, temp_dir: str, package_info: Dict[str, Any], package_name: str
    ) -> str:
        """Create a setup.py file for the package."""
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
    install_requires=[
        # Add any Python dependencies here
        # Note: ROS dependencies should be handled separately
    ],
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

        # Step 5: Create temporary build directory
        with tempfile.TemporaryDirectory() as temp_dir:
            print(f"Using temporary directory: {temp_dir}")

            # Copy the Python package to temp directory
            package_dest = os.path.join(temp_dir, ros_package)
            try:
                shutil.copytree(python_package_path, package_dest)
                print(f"Copied package to: {package_dest}")
            except Exception as e:
                print(f"Error copying package: {e}")
                return False

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

    args = parser.parse_args()

    if args.verbose:
        print(f"Building wheel for: {args.package}")
        print(f"ROS distro: {args.ros_distro}")
        print(f"Output directory: {args.output_dir}")

    builder = ROSPythonPackageBuilder(ros_distro=args.ros_distro)
    success = builder.build_wheel(args.package, args.output_dir)

    if success:
        print(f"Successfully built wheel for {args.package}")
        sys.exit(0)
    else:
        print(f"Failed to build wheel for {args.package}")
        sys.exit(1)


if __name__ == "__main__":
    main()
