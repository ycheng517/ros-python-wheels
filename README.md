# ROS Python Wheels

Convert ROS packages into Python wheels that can be installed via pip.

## Benefits

Enable a first-class developer experience when working with ROS in Python projects:

- **Easy Project Integration**: To include The ROS Python Client ([rclpy](https://github.com/ros2/rclpy)) in a Python project, simply add `ros-rclpy[fastrtps]` with its package repository to your `pyproject.toml` file.
- **Enable Modern Python Tooling**: Easily manage Python ROS dependencies using modern Python tools like uv, Poetry, and conda
- **Lightweight**: The wheels of rclpy all of its dependencies have a total size of around 15MB.
- **Portable**: Allows ROS to be run on different Linux distributions.

## Current Limitations

- The Python version must match the Python version for a ROS distribution.
- Only works for `x86_64-linux-gnu` systems. Linux distributions using GNU includes Ubuntu, Debian, Fedora, and Arch Linux.

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
