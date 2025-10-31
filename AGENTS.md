# Instructions for Agents

This document provides instructions for AI agents working in the `ros-python-wheels-manylinux` codebase.

## Project Overview

This project builds ROS 2 packages into `manylinux` compatible wheels. It can recursively build all dependencies of a given ROS package, and it handles both C++ (CMake) and Python packages.

## Key Technologies

- **Python:** The core logic is written in Python.
- **ROS 2:** The project is designed to work with the ROS 2 ecosystem.
- **`cibuildwheel`:** This tool is used to build C++ packages into wheels.
- **`fire`:** This library is used for creating the command-line interface.
- **`jinja2`:** This is used for templating files for the build process.
- **`uv`:** This is used for managing the virtual environment.

## How it Works

The build process is as follows:

1.  **Dependency Resolution**: The tool takes a target ROS package and a ROS distribution as input. It recursively finds all ROS dependencies (both build and run dependencies) for the target package.

2.  **Build Order Generation**: It then creates a dependency graph of all the ROS packages to be built and generates a topological sort of this graph to determine the correct build order.

3.  **Package Building**: It iterates through the build order and builds each package one by one.

    - For **C++ (CMake) packages**, it generates a Python wrapper (`pyproject.toml`, `setup.py`, a dummy C file) and then uses `cibuildwheel` to build a `manylinux` compatible wheel. The `cibuildwheel` process is configured to install system dependencies (like `ninja-build`) and Python dependencies before building the wheel.
    - For **Python packages**, it uses the standard `build` tool to create a wheel. It modifies the package name in `pyproject.toml` or `setup.py` to follow a `ros-<package-name>` convention.

4.  **Meta Package Creation**: After building a package, it creates a "meta" package. This meta package has the version of the original package and has a dependency on the wheel that was just built. This allows for easy installation of the correct version of the package for a given ROS distribution. For example, `pip install ros-humble-rclpy` will install the `rclpy` wheel that was built for the `humble` distribution.

## How to Run a Build

To run a build, you first need to source the virtual environment:

```bash
source .venv/bin/activate
```

Then, you can use the `build-ros-wheel` script. For example, to build the `rclpy` package for the `jazzy` distribution, you would run:

```bash
python -m ros_wheel_builder.main --package_name rclpy --distro_name jazzy
```

You can also do a dry run to see the build order without actually building anything:

```bash
python -m ros_wheel_builder.main --package_name rclpy --distro_name jazzy --print-only
```

## How to Upload a Wheel

To upload wheels, you can use the `upload-ros-wheel` script. The script now includes tracking functionality to avoid re-uploading packages, which is especially useful for repositories like Cloudsmith that don't support `--skip-existing`.

### Upload all wheels in the artifacts directory:

```bash
python -m ros_wheel_builder.upload upload --repository cloudsmith
```

### List already uploaded packages:

```bash
python -m ros_wheel_builder.upload list --repository cloudsmith
```

### Clear the uploaded packages tracking (useful for testing):

```bash
python -m ros_wheel_builder.upload clear --repository cloudsmith
```

### Remove a specific package from the uploaded tracking:

```bash
python -m ros_wheel_builder.upload remove --repository cloudsmith --package-filename "ros-package-1.0.0-py3-none-any.whl"
```

The upload tracking creates `.uploaded_packages_<repository>.json` files to track which packages have been successfully uploaded to each repository. This prevents duplicate uploads and saves time on subsequent runs.

## The ros_wheel_builder modules

- **`build.py`**: Contains functions to generate `pyproject.toml`, `setup.py`, `dummy.c` and `repair_wheel.sh` files for C++ packages, and a function to generate `setup.py` for meta packages.
- **`dependency_resolver.py`**: Contains a function to generate a build order for a list of packages.
- **`distro.py`**: Contains a `Distro` class that provides information about a ROS 2 distribution, including dependencies.
- **`main.py`**: Contains the main logic for building ROS 2 packages into wheels. It uses the other modules to perform the build.
- **`ros_distro.py`**: Contains functions to download and parse ROS 2 distribution files.
- **`upload.py`**: Contains functions to upload wheels to a repository, and to manage the list of uploaded packages.

## Virtual Environment

The venv is managed by `uv`. To install dependencies, use `uv pip`:

```bash
uv pip install <package_name>
```
