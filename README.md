# ROS Python Wheels

Convert ROS packages into Python wheels that can be installed via pip.

## Benefits

Enable a first-class developer experience when working with ROS in Python projects:

- **Easy Project Integration**: To include The ROS Python Client ([rclpy](https://github.com/ros2/rclpy)) in a Python project, simply add `ros-rclpy[fastrtps]` with its package repository to your `pyproject.toml` file.
- **Enable Modern Python Tooling**: Easily manage Python ROS dependencies using modern Python tools like uv, Poetry, and conda.
  - Check for vulnerabilities of ROS Python packages using PyPI scanners like [socket.dev/](https://socket.dev/search?e=pypi).
- **Lightweight**: The wheels of rclpy all of its dependencies have a total size of around 15MB.
  - A docker image with rclpy based on `python:3.12-alpine` only takes 263 MB, compared with 875MB for `ros:jazzy-ros-base`
- **Portable**: Allows ROS to be run on different Linux distributions. The only requirement for now is `x86_64`.

## Current Limitations

- The Python version must match the Python version for a ROS distribution.
- Only works for `x86_64` on Linux for now.

## Python Package Repositories

Packages are hosted at [Cloudsmith](https://cloudsmith.com), where there's a separate repository for each of the currently supported ROS distributions.

- Kilted: https://dl.cloudsmith.io/public/ros-python-wheels/kilted/python/simple/
- Jazzy: https://dl.cloudsmith.io/public/ros-python-wheels/jazzy/python/simple/
- Humble: https://dl.cloudsmith.io/public/ros-python-wheels/humble/python/simple/

### Available Packages

Open each package repository link in your browser to see the list of packages it contains.

## Example Usage

Below steps will install and run the ROS Python client for Kilted in a Python 3.12 environment.

```bash
# install rclpy and dependencies
pip install \
  --extra-index-url https://dl.cloudsmith.io/public/ros-python-wheels/kilted/python/simple/ \
  ros-rclpy[fastrtps]

# set LD_LIBRARY_PATH to the ros_runtime_libs/ directory of site-packages
# where ros-rclpy is installed
export LD_LIBRARY_PATH=$(pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs
# Set the RMW to use
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Run rclpy
python -c "import rclpy; rclpy.init()"
```

## Building Wheels

The following script builds and uploads packages to Cloudsmith for a given ROS distribution. Cloudsmith credentials are required.

```bash
./scripts/run_all.sh <ROS_DISTRO>
```
