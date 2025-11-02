import fire
import os
import subprocess
import pprint
import shutil
from pathlib import Path

from . import file_gen
from .dependency_resolver import generate_build_order
from .distro import Distro, PackageSource
from .ros_distro import get_distro_file, find_package, download_source


def format_ros_package_name(package_name: str) -> str:
    """Convert a ROS package name to the wheel format: ros-package"""
    return f"ros-{package_name.replace('_', '-')}"


def get_recursive_build_depends(
    distro: Distro, pkg_name: str
) -> tuple[set[str], set[str], set[str]]:
    """Get all recursive dependencies for a set of packages."""
    queue = [pkg_name]
    processed = set()
    all_ros_deps: set[str] = set()
    all_python_deps: set[str] = set()
    all_system_deps: set[str] = set()
    while queue:
        package = queue.pop(0)
        if package in processed:
            continue
        processed.add(package)

        deps = distro.get_build_depends(package)
        ros_deps = {d.dep_name for d in deps if d.source == PackageSource.ROS}
        python_deps = {d.dep_name for d in deps if d.source == PackageSource.PYTHON}
        system_deps = {d.dep_name for d in deps if d.source == PackageSource.SYSTEM}

        for dep in ros_deps:
            all_ros_deps.add(dep)
            queue.append(dep)
        for dep in python_deps:
            all_python_deps.add(dep)
        for dep in system_deps:
            all_system_deps.add(dep)

    return all_ros_deps, all_python_deps, all_system_deps


def apply_patch(package_name, distro_name, src_dir):
    patch_dir = Path("patches") / distro_name
    patch_file = patch_dir / f"{package_name}.patch"
    if not patch_file.exists():
        return

    package_dir = src_dir / package_name

    # Check if patch is already applied by doing a dry run first
    # Try a dry run first to see if patch would apply cleanly
    dry_run = subprocess.run(
        ["git", "apply", "--check", str(patch_file.resolve())],
        capture_output=True,
        cwd=package_dir,
    )

    if dry_run.returncode == 0:
        # Patch can be applied, so apply it
        print(f"Applying patch for {package_name} in {distro_name}")
        subprocess.run(
            ["git", "apply", str(patch_file.resolve())],
            check=True,
            cwd=package_dir,
        )
    else:
        # Check if patch is already applied by trying reverse patch
        reverse_check = subprocess.run(
            ["git", "apply", "--check", "--reverse", str(patch_file.resolve())],
            capture_output=True,
            cwd=package_dir,
        )

        if reverse_check.returncode == 0:
            print(
                f"Patch for {package_name} in {distro_name} is already applied, skipping"
            )
        else:
            print(
                f"Warning: Patch for {package_name} in {distro_name} cannot be applied and is not already applied"
            )
            print(
                f"Patch check failed with: {dry_run.stderr.decode() if dry_run.stderr else 'Unknown error'}"
            )


def get_extras_require(package_name: str, distro: Distro) -> dict | None:
    if package_name == "rclpy":
        dep_version = distro.get_version("rmw_fastrtps_cpp")
        return {"fastrtps": [f"ros-rmw-fastrtps-cpp~={dep_version}"]}
    return None


def build_package(
    package_name,
    distro: Distro,
    distro_data,
    src_dir,
    artifacts_dir,
):
    package_dir = src_dir / package_name

    # Download source
    if not package_dir.exists():
        package_info = find_package(distro_data, package_name)
        if package_info is None:
            raise ValueError(
                f"Package {package_name} not in distro {distro.distro_name}"
            )
        download_source(package_name, package_info, src_dir)

    apply_patch(package_name, distro.distro_name, src_dir)

    is_cmake_package = (package_dir / "CMakeLists.txt").exists()
    if is_cmake_package:
        print(f"{package_name} is a C++ package, generating build shim...")

        # Get the specific build dependencies for this package
        build_ros_deps, build_python_deps, build_system_deps = (
            get_recursive_build_depends(distro, package_name)
        )
        # Assume we've already built it, so now it's a python dep
        for dep_name in build_ros_deps:
            formatted_dep_name = format_ros_package_name(dep_name)
            build_python_deps.add(formatted_dep_name)

        version = distro.get_version(package_name)
        run_deps = distro.get_run_depends(package_name)
        run_dependency_names = []
        all_system_deps = build_system_deps
        for dep in run_deps:
            if dep.source == PackageSource.ROS:
                # Format ROS dependencies with the new naming convention
                formatted_name = format_ros_package_name(dep.dep_name)

                # Get the version of the dependency
                dep_version = distro.get_version(dep.dep_name)

                # Add the version constraint
                run_dependency_names.append(f"{formatted_name}~={dep_version}")
            elif dep.source == PackageSource.PYTHON:
                # Keep Python dependencies as-is
                run_dependency_names.append(dep.dep_name)
            elif dep.source == PackageSource.SYSTEM:
                all_system_deps.add(dep.dep_name)
        if package_name == "rclpy":
            run_dependency_names.append("ros-wheels-bootstrapper")
        extras_require = get_extras_require(package_name, distro)

        # Extract README information
        description, long_description = file_gen.get_package_description(
            package_name, package_dir
        )

        setup_py_content = file_gen.generate_cpp_setup_py(
            package_name,
            version,
            package_name,
            run_dependency_names,
            extras_require=extras_require,
            description=description,
            long_description=long_description,
        )
        with open(package_dir / "setup.py", "w") as f:
            f.write(setup_py_content)

        # Add ninja-build as it's always needed for C++ packages
        all_system_deps.add("ninja-build")
        all_system_deps.add("zip")
        # cmake is already in the image
        if "cmake3" in all_system_deps:
            all_system_deps.remove("cmake3")
        # use the third party version for fastrtps
        if "asio-devel" in all_system_deps:
            all_system_deps.remove("asio-devel")

        # Pin empy to a compatible version
        if "empy" in build_python_deps:
            build_python_deps.remove("empy")
            build_python_deps.add("empy==3.3.4")

        build_python_deps |= {"setuptools", "wheel"}
        before_build_cmd = "pipx install cmake==3.28.3"
        if all_system_deps:
            before_build_cmd += (
                " && echo 'fastestmirror=true' >> /etc/dnf/dnf.conf && dnf install -y "
                + " ".join(all_system_deps)
            )
        before_build_cmd += " && pip install " + " ".join(build_python_deps)

        pyproject_toml_content = file_gen.generate_cpp_pyproject_toml(before_build_cmd)
        with open(package_dir / "pyproject.toml", "w") as f:
            f.write(pyproject_toml_content)

        dummy_c_content = file_gen.generate_dummy_c()
        with open(package_dir / "dummy.c", "w") as f:
            f.write(dummy_c_content)

        # Copy all .whl files from artifacts_dir to wheelhouse_dir
        wheelhouse_dir = package_dir / "wheelhouse"
        wheelhouse_dir.mkdir(exist_ok=True)
        for wheel_file in artifacts_dir.glob("*.whl"):
            destination_path = wheelhouse_dir / wheel_file.name
            if not (
                destination_path.exists() and destination_path.samefile(wheel_file)
            ):
                shutil.copy2(wheel_file, wheelhouse_dir)

        # Create the custom repair script
        repair_script_content = file_gen.generate_repair_script()
        repair_script_path = package_dir / "repair_wheel.sh"
        with open(repair_script_path, "w") as f:
            f.write(repair_script_content)
        os.chmod(repair_script_path, 0o755)

        repair_command = "/project/repair_wheel.sh {wheel} {dest_dir}"
        env = {
            **os.environ,
            "CIBW_ENVIRONMENT": "PIP_FIND_LINKS=/project/wheelhouse",
            "CIBW_REPAIR_WHEEL_COMMAND": repair_command,
            "CIBW_BUILD_FRONTEND": "build; args: --no-isolation",
        }

        subprocess.run(
            [
                "cibuildwheel",
                "--output-dir",
                str(artifacts_dir.resolve()),
            ],
            check=True,
            env=env,
            cwd=package_dir,
        )
    else:
        # build python package using wheel
        print(f"{package_name} is a Python package, building wheel...")

        # Update the package name in pyproject.toml or setup.py to use the ROS naming convention
        pkg_name_fmt = format_ros_package_name(package_name)

        # Check for pyproject.toml first
        pyproject_path = package_dir / "pyproject.toml"
        setup_py_path = package_dir / "setup.py"

        if pyproject_path.exists():
            # Update pyproject.toml
            with open(pyproject_path, "r") as f:
                content = f.read()

            # Replace name = "original_name" with name = "ros-distro-name"
            import re

            content = re.sub(
                r'^(\s*name\s*=\s*)["\']([^"\']+)["\']',
                rf'\g<1>"{pkg_name_fmt}"',
                content,
                flags=re.MULTILINE,
            )

            with open(pyproject_path, "w") as f:
                f.write(content)

        elif setup_py_path.exists():
            # Update setup.py
            with open(setup_py_path, "r") as f:
                content = f.read()

            # Replace name="original_name" or name='original_name' with the formatted name
            import re

            # First, handle direct string literals: name = "package_name"
            content = re.sub(
                r'^(\s*name\s*=\s*)["\']([^"\']+)["\']',
                rf'\g<1>"{pkg_name_fmt}"',
                content,
                flags=re.MULTILINE,
            )

            # Then, handle when name=package_name (variable reference) - replace with direct string
            content = re.sub(
                r"^(\s*name\s*=\s*)package_name\s*,?\s*$",
                rf'\g<1>"{pkg_name_fmt}",',
                content,
                flags=re.MULTILINE,
            )

            with open(setup_py_path, "w") as f:
                f.write(content)

        subprocess.run(
            [
                "python3",
                "-m",
                "build",
                "--wheel",
                "--outdir",
                str(artifacts_dir.resolve()),
            ],
            check=True,
            cwd=package_dir,
        )


def build_meta_package(
    package_name,
    distro_name,
    version,
    artifacts_dir,
    extras_require=None,
):
    """
    Builds a meta package that depends on the built package.
    """
    meta_package_name = f"ros-{distro_name}-{package_name.replace('_', '-')}"
    dependency_name = f"ros-{package_name.replace('_', '-')}"

    meta_pkg_dir = Path("build") / distro_name / "meta" / meta_package_name
    meta_pkg_dir.mkdir(parents=True, exist_ok=True)

    description, long_description = file_gen.get_meta_package_description(
        package_name, distro_name
    )
    setup_py_content = file_gen.generate_meta_package_setup_py(
        meta_package_name,
        version,
        dependency_name,
        version,
        extras_require=extras_require,
        description=description,
        long_description=long_description,
    )
    with open(meta_pkg_dir / "setup.py", "w") as f:
        f.write(setup_py_content)

    pyproject_toml_content = file_gen.generate_cpp_pyproject_toml("")
    with open(meta_pkg_dir / "pyproject.toml", "w") as f:
        f.write(pyproject_toml_content)

    subprocess.run(
        [
            "python3",
            "-m",
            "build",
            "--wheel",
            "--outdir",
            str(artifacts_dir.resolve()),
        ],
        check=True,
        cwd=meta_pkg_dir,
    )


def build(
    distro_name: str,
    package_name: str,
    print_only: bool = False,
    skip_existing: bool = False,
):
    """
    Build a ROS 2 package into a manylinux wheel.

    Args:
        distro_name (str): The ROS 2 distribution to build for.
        package_name (str): The name of the ROS 2 package to build.
        print_only (bool): If True, only print the build order without building.
        skip_existing (bool): If True, skip packages that have already been built.
    """
    print(f"Building {package_name} for {distro_name}")

    build_dir = Path("build") / distro_name
    src_dir = build_dir / "src"
    src_dir.mkdir(parents=True, exist_ok=True)
    artifacts_dir = build_dir / "artifacts"
    artifacts_dir.mkdir(exist_ok=True)

    if package_name == "rclpy" and not print_only:
        bootstrapper_dir = Path("ros-wheels-bootstrapper")
        if bootstrapper_dir.exists():
            print("--- Building ros-wheels-bootstrapper ---")
            subprocess.run(
                [
                    "python3",
                    "-m",
                    "build",
                    "--wheel",
                    "--outdir",
                    str(artifacts_dir.resolve()),
                    str(bootstrapper_dir.resolve()),
                ],
                check=True,
            )

    distro = Distro(distro_name)
    distro_data = get_distro_file(distro_name)

    # Collect all packages that need to be built (build + run deps + target)
    print("--- Collecting all dependencies ---")

    # Get all dependencies recursively
    all_ros_deps = set()
    all_python_deps = set()
    all_system_deps = set()

    # Start with the target package
    packages_to_analyze = {package_name}
    analyzed_packages = set()

    while packages_to_analyze:
        current_package = packages_to_analyze.pop()
        if current_package in analyzed_packages:
            continue
        analyzed_packages.add(current_package)

        # Get both build and run dependencies for this package
        build_deps = distro.get_build_depends(current_package)
        run_deps = distro.get_run_depends(current_package)
        all_deps = build_deps | run_deps

        for dep in all_deps:
            if dep.source == PackageSource.ROS:
                if dep.dep_name not in all_ros_deps:
                    all_ros_deps.add(dep.dep_name)
                    packages_to_analyze.add(dep.dep_name)
            elif dep.source == PackageSource.PYTHON:
                all_python_deps.add(dep.dep_name)
            elif dep.source == PackageSource.SYSTEM:
                all_system_deps.add(dep.dep_name)
    all_ros_deps |= {package_name}

    print("All ROS packages:")
    pprint.pprint(all_ros_deps)
    print("All Python dependencies:")
    pprint.pprint(all_python_deps)
    print("All System dependencies:")
    pprint.pprint(all_system_deps)

    # Generate build order for all packages including target
    build_order = generate_build_order(list(all_ros_deps), distro)
    print("Build order:")
    pprint.pprint(build_order)

    if print_only:
        return
    for i, pkg_name in enumerate(build_order):
        pkg_name_fmt = format_ros_package_name(pkg_name)
        pkg_name_fmt_ = pkg_name_fmt.replace("-", "_")
        if skip_existing and (
            any(artifacts_dir.glob(f"{pkg_name_fmt}-*.whl"))
            or any(artifacts_dir.glob(f"{pkg_name_fmt_}-*.whl"))
        ):
            print(f"Skipping {pkg_name}, already built.")
            continue

        print(f"Building {pkg_name} ({i + 1}/{len(build_order)})")
        build_package(
            pkg_name,
            distro,
            distro_data,
            src_dir,
            artifacts_dir,
        )

        version = distro.get_version(pkg_name)
        extras_require = get_extras_require(pkg_name, distro)
        build_meta_package(
            pkg_name,
            distro_name,
            version,
            artifacts_dir,
            extras_require=extras_require,
        )


def info(distro_name: str, package_name: str):
    """
    Get information about a ROS 2 package.

    Args:
        distro_name (str): The ROS 2 distribution to get information from.
        package_name (str): The name of the ROS 2 package.
    """
    distro = Distro(distro_name)

    print(f"Package: {package_name}")
    print(f"Distribution: {distro_name}")
    print("")

    build_depends = distro.get_build_depends(package_name)
    print("Build Dependencies:")
    pprint.pprint(build_depends)
    print("")

    run_depends = distro.get_run_depends(package_name)
    print("Run Dependencies:")
    pprint.pprint(run_depends)
    print("")

    test_depends = distro.get_test_depends(package_name)
    print("Test Dependencies:")
    pprint.pprint(test_depends)
    print("")

    repo_url, repo_version = distro.get_released_repo(package_name)
    print("Repository URL:", repo_url)
    print("Repository Version:", repo_version)


def main():
    """Entry point for the build-ros-wheel command."""
    fire.Fire(
        {
            "build": build,
            "info": info,
        }
    )


if __name__ == "__main__":
    main()
