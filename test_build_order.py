#!/usr/bin/env python3
"""
Test script to verify the build order functionality works correctly.
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(__file__))

from build_python_package import ROSPythonPackageBuilder


def test_build_order(package_name: str):
    """Test the build order determination for a package."""
    print(f"Testing build order for: {package_name}")

    builder = ROSPythonPackageBuilder(ros_distro="jazzy")

    try:
        build_order = builder.get_build_order(package_name)

        if build_order:
            print(f"✅ Build order determined ({len(build_order)} packages):")
            for i, pkg in enumerate(build_order, 1):
                print(f"  {i}. {pkg}")
        else:
            print(f"❌ No Python packages found for {package_name}")

    except Exception as e:
        print(f"❌ Error determining build order: {e}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 test_build_order.py <package_name>")
        sys.exit(1)

    test_build_order(sys.argv[1])
