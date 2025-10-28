# Instructions for Agents

This document provides instructions for AI agents working in the `ros-python-wheels-manylinux` codebase.

## Project Overview

This project builds ROS 2 packages into `manylinux` compatible wheels. It can recursively build all dependencies of a given ROS package, and it handles both C++ (CMake) and Python packages.

## Key Technologies

*   **Python:** The core logic is written in Python.
*   **ROS 2:** The project is designed to work with the ROS 2 ecosystem.
*   **`cibuildwheel`:** This tool is used to build C++ packages into wheels.
*   **`fire`:** This library is used for creating the command-line interface.
*   **`jinja2`:** This is used for templating files for the build process.
*   **`uv`:** This is used for managing the virtual environment.

## How it Works

The build process is divided into two stages:

1.  **Stage 1: Build Dependencies:** The tool first identifies and builds all the *build-time* dependencies of the target ROS package.
2.  **Stage 2: Run Dependencies and Target Package:** After the build dependencies are built, the tool builds the *run-time* dependencies and the target package itself.

For C++ packages, the tool generates a Python wrapper (`pyproject.toml`, `setup.py`, etc.) and then uses `cibuildwheel` to create the wheel. For Python packages, it uses the standard `build` tool.

## How to Run a Build

To run a build, you first need to source the virtual environment:

```bash
source .venv/bin/activate
```

Then, you can use the `build-ros-wheel` script. For example, to build the `rclpy` package for the `humble` distribution, you would run:

```bash
python -m ros_wheel_builder.main humble rclpy
```

You can also do a dry run to see the build order without actually building anything:

```bash
python -m ros_wheel_builder.main humble rclpy --print-only
```

## How to Upload a Wheel

To upload a wheel, you can use the `upload-ros-wheel` script:

```bash
python -m ros_wheel_builder.upload <distro_name> <package_name>
```

## Virtual Environment

The venv is managed by `uv`. To install dependencies, use `uv pip`:

```bash
uv pip install <package_name>
```