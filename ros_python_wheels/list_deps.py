import argparse
import os
import subprocess
from catkin_pkg.package import parse_package


def resolve_rosdep_to_debian(package_name: str) -> str:
    """
    Use rosdep resolve to get the debian package name for a ROS package.
    Returns the debian package name or the original name if resolution fails.
    """
    try:
        result = subprocess.run(
            ["rosdep", "resolve", package_name],
            capture_output=True,
            text=True,
            check=True,
        )
        # rosdep resolve output format is typically "#apt\npackage-name"
        lines = result.stdout.strip().split("\n")
        for line in lines:
            if line.startswith("#"):
                continue
            if line.strip():
                return line.strip()
        return package_name
    except (subprocess.CalledProcessError, FileNotFoundError):
        # If rosdep resolve fails, return the original name
        return package_name


def categorize_ros_package(debian_package_name: str) -> str:
    """
    Categorize a ROS package by checking its file contents using dpkg -L.
    Returns: 'python', 'runtime', or 'other'
    """
    try:
        result = subprocess.run(
            ["dpkg", "-L", debian_package_name],
            capture_output=True,
            text=True,
            check=True,
        )
        files = result.stdout.strip().split("\n")

        has_python_content = False
        has_runtime_content = False

        for file_path in files:
            # Check for Python packages
            if "/opt/ros/jazzy/lib/python3.12/site-packages/" in file_path:
                has_python_content = True
            # Check for runtime libraries (.so files)
            elif file_path.startswith("/opt/ros/jazzy/lib/") and (
                file_path.endswith(".so") or ".so." in file_path
            ):
                has_runtime_content = True

        # Python packages take priority (even if they also have .so files)
        if has_python_content:
            return "python"
        elif has_runtime_content:
            return "runtime"
        else:
            return "other"

    except (subprocess.CalledProcessError, FileNotFoundError):
        # If dpkg -L fails, default to 'other'
        return "other"


def get_deps(
    package_name: str,
    ros_python_deps: set[str],
    ros_runtime_deps: set[str],
    ros_other_deps: set[str],
    python_deps: set[str],
    system_deps: set[str],
    processed: set[str],
    level: int = 0,
    recurse: bool = True,
):
    """
    Recursively finds all dependencies for a ROS package using catkin package.xml.

    Categorizes deps into ROS (python/runtime/other), Python, and system dependencies.
    Only recurses into ROS dependencies.

    Args:
        package_name: The name of the ROS package to analyze.
        ros_python_deps: Set to collect ROS Python dependencies.
        ros_runtime_deps: Set to collect ROS runtime dependencies.
        ros_other_deps: Set to collect other ROS dependencies.
        python_deps: Set to collect Python dependencies.
        system_deps: Set to collect system dependencies.
        processed: Set to keep track of processed packages to avoid cycles.
        level: Current recursion level for indentation.
        recurse: Whether to recurse into ROS dependencies.
    """
    if package_name in processed:
        return

    processed.add(package_name)

    try:
        package_path = os.path.join("/opt/ros/jazzy/share", package_name, "package.xml")
        if not os.path.exists(package_path):
            return

        pkg = parse_package(package_path)

        # Combine build and exec dependencies
        all_deps = list(pkg.exec_depends)

        for dep in all_deps:
            dep_name = dep.name

            if dep_name in processed:
                continue

            # Resolve to debian package name
            debian_name = resolve_rosdep_to_debian(dep_name)

            # Categorize the dependency
            if debian_name.startswith("ros-"):
                ros_category = categorize_ros_package(debian_name)
                if ros_category == "python":
                    category = "ros-python"
                    target_set = ros_python_deps
                elif ros_category == "runtime":
                    category = "ros-runtime"
                    target_set = ros_runtime_deps
                else:
                    category = "ros-other"
                    target_set = ros_other_deps
            elif debian_name.startswith("python3"):
                category = "python"
                target_set = python_deps
            else:
                category = "system"
                target_set = system_deps

            if dep_name not in target_set:
                target_set.add(dep_name)
                indent = "  " * level
                print(f"{indent}└─ {dep_name} -> {debian_name} ({category})")

                # Only recurse into ROS dependencies if recursion is enabled
                if recurse and debian_name.startswith("ros-"):
                    get_deps(
                        dep_name,
                        ros_python_deps,
                        ros_runtime_deps,
                        ros_other_deps,
                        python_deps,
                        system_deps,
                        processed,
                        level + 1,
                        recurse,
                    )

    except Exception as e:
        indent = "  " * level
        print(f"{indent}└─ Error processing {package_name}: {e}")


def parse_catkin_package(package_name: str, recurse: bool = True):
    """
    Parse a catkin package and show all its dependencies categorized.
    """
    try:
        package_path = os.path.join("/opt/ros/jazzy/share", package_name, "package.xml")
        if not os.path.exists(package_path):
            print(f"Error: Package '{package_name}' not found at {package_path}")
            return

        # Sets to keep track of different types of dependencies
        ros_python_deps: set[str] = set()
        ros_runtime_deps: set[str] = set()
        ros_other_deps: set[str] = set()
        python_deps: set[str] = set()
        system_deps: set[str] = set()
        processed: set[str] = set()

        get_deps(
            package_name,
            ros_python_deps,
            ros_runtime_deps,
            ros_other_deps,
            python_deps,
            system_deps,
            processed,
            recurse=recurse,
        )

        print("\n🐍 ROS Python Packages:")
        for dep in sorted(ros_python_deps):
            print(f"  - {dep}")

        print("\n⚡ ROS Runtime Packages:")
        for dep in sorted(ros_runtime_deps):
            print(f"  - {dep}")

        print("\n📦 Other ROS Packages:")
        for dep in sorted(ros_other_deps):
            print(f"  - {dep}")

        print("\n🐍 Python Dependencies:")
        for dep in sorted(python_deps):
            print(f"  - {dep}")

        print("\n⚙️  System Dependencies:")
        for dep in sorted(system_deps):
            print(f"  - {dep}")

    except Exception as e:
        print(f"Error parsing package '{package_name}': {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Parse a Catkin package and categorize its dependencies"
    )
    parser.add_argument("package_name", help="The name of the Catkin package to parse")
    parser.add_argument(
        "--no-recurse",
        action="store_true",
        help="Disable recursive dependency resolution (only show direct dependencies)",
    )
    args = parser.parse_args()

    parse_catkin_package(args.package_name, recurse=not args.no_recurse)
