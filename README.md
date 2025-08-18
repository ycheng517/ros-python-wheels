# ROS Python Wheels

Convert ROS packages into Python wheels that can be installed via pip.

## Benefits

- **Easy Project Integration**: To include The ROS Python Client ([rclpy](https://github.com/ros2/rclpy)) in a Python project, simply add `ros-rclpy[fastrtps]` to your `pyproject.toml`
  - Fully manage Python ROS dependencies using modern Python tooling like uv, Poetry, and conda.
- **Lightweight**: rclpy all of its dependencies takes up less than 20MB.
  - A docker image with rclpy based on `python:3.12` only takes XXX MB, compared with 875MB for `ros:jazzy-ros-base`
- **Portable**: Allows ROS to be run on different Linux distros. The only requirement is `x86_64`.

## Example: Install and Run the ROS Python Client

```bash
# install rclpy
pip install ros-rclpy[fastrtps]

# set LD_LIBRARY_PATH to the ros_runtime_libs/ directory of site-packages
# where ros-rclpy is installed
export LD_LIBRARY_PATH=$(pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs
# Set the RMW to use
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Run rclpy
python -c "import rclpy; rclpy.init()"
```

## Building Wheels

### Pre-requisites

You need to have the ROS version that you'd like to build wheels for installed. For example,
to build wheels for ROS 2 Jazzy, you need to have ROS 2 Jazzy installed on your system.

### Usage

Install this package and activate the environment using your favorite Python
package/environment manager, i.e.

```bash
uv sync
source .venv/bin/activate
```

Build a ROS package and all of its dependencies to the `dist/` directory

```bash
python scripts/build.py test_msgs
```

Can build a list of ROS packages as well:

```bash
python scripts/build.py test_msgs rmw_fastrtps_cpp,rclpy,test_msgs
```
