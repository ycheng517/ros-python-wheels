# ROS Python Wheels

Convert ROS packages into Python wheels that can be installed via pip. A select number of packages are hosted on [PyPI](https://pypi.org/) and [Cloudsmith](https://cloudsmith.com).

## Benefits

Enable a first-class developer experience when working with ROS in Python projects:

- **Easy Project Integration**: To include The ROS Python Client ([rclpy](https://github.com/ros2/rclpy)) in a Python project, simply add `ros-rclpy[fastrtps]` with its package repository to your `pyproject.toml` file.
- **Enable Modern Python Tooling**: Easily manage Python ROS dependencies using modern Python tools like uv, Poetry, and conda
- **Portable**: Allows ROS to be run on different Linux distributions.

## Python Package Repositories

Packages are hosted at https://pypi.org/simple/ and https://dl.cloudsmith.io/public/ros-python-wheels/main/python/simple/

### Available Packages

Open each package repository link in your browser to see the list of packages it contains.
Currently `builtin_interfaces`, `rclpy` and their dependencies are uploaded.
Please create a Github issue if you'd like to see another package built!

## Example Usage

Below steps will install and run the ROS Python client for Kilted in a Python 3.12 environment.

```bash
# install rclpy and dependencies
pip install \
  --extra-index-url https://dl.cloudsmith.io/public/ros-python-wheels/main/python/simple/ \
  ros-rclpy[fastrtps]
# Run rclpy
python -c "import rclpy; rclpy.init()"
```

## Building and Uploading Wheels

The following commands build a package and all of its dependencies as wheels, and then upload them to CloudSmith.
Cloudsmith credentials are required in `~/.pypirc`.

```bash
# Install this project
uv sync
# Build wheels
python -m ros_wheel_builder.main build --distro_name <DISTRO> --package_name <PACKAGE_NAME>
# Upload wheels
python -m ros_wheel_builder.upload --wheels_dir <WHEELS_DIR> --repository <pypi/testpypi/cloudsmith>
```
