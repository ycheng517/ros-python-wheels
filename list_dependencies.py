import apt
import sys


def get_all_deps(package, cache, ros_deps, python_deps, system_deps, level=0):
    """
    Recursively finds all dependencies for a given package.
    Categorizes deps into ROS, Python, and system dependencies.
    Only recurses into ROS dependencies.
    """
    # This is the line that gets the list of required dependencies
    for dep_list in package.versions[0].dependencies:
        # A dependency can be a choice, e.g., "libssl3 | libssl1.1".
        # We'll just check the first option in the list for this example.
        dep = dep_list[0]

        # Categorize the dependency
        if dep.name.startswith("ros-"):
            category = "ros"
            target_set = ros_deps
        elif dep.name.startswith("python3"):
            category = "python"
            target_set = python_deps
        else:
            category = "system"
            target_set = system_deps

        # Avoid printing duplicates or getting into circular dependency loops
        if dep.name in target_set:
            continue

        target_set.add(dep.name)
        indent = "  " * level
        print(f"{indent}└─ {dep.name} ({category})")

        try:
            # Get the actual package object for the dependency to recurse
            dep_pkg = cache[dep.name]
            # Only recurse into ROS dependencies
            if category == "ros":
                get_all_deps(
                    dep_pkg, cache, ros_deps, python_deps, system_deps, level + 1
                )
        except KeyError:
            # This can happen for virtual packages.
            pass


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <package-name>")
        sys.exit(1)

    package_name = sys.argv[1]

    try:
        cache = apt.Cache()
        # It's good practice to update the cache view if it might be stale
        # cache.update()
        cache.open()

        initial_package = cache[package_name]

        print(f"Dependencies for: {initial_package.name} 🌳")

        # Three sets to keep track of different types of dependencies
        ros_deps: set[str] = set()
        python_deps: set[str] = set()
        system_deps: set[str] = set()

        get_all_deps(initial_package, cache, ros_deps, python_deps, system_deps)

        print("\n🤖 ROS Dependencies:")
        for dep in sorted(ros_deps):
            print(f"  - {dep}")

        print("\n🐍 Python Dependencies:")
        for dep in sorted(python_deps):
            print(f"  - {dep}")

        print("\n⚙️  System Dependencies:")
        for dep in sorted(system_deps):
            print(f"  - {dep}")

    except KeyError:
        print(f"Error: Package '{package_name}' not found in APT cache.")
    except Exception as e:
        print(f"An error occurred: {e}")
