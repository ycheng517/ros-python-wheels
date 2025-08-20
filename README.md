# ROS Python Wheels

Convert ROS packages into Python wheels that can be installed via pip.

## Benefits

Here are some benefits unlocked by using Python wheels of ROS packages:

- **Easy Project Integration**: To include The ROS Python Client ([rclpy](https://github.com/ros2/rclpy)) in a Python project, simply add `ros-rclpy[fastrtps]` to your `pyproject.toml` file.
- **Enable Modern Python Tooling**: Easily manage Python ROS dependencies using modern Python tools like uv, Poetry, and conda.
  - Check for vulnerabilities of ROS Python packages using tools like [socket.dev/](https://socket.dev/search?e=pypi) and pip-audit.
- **Lightweight**: The wheels of rclpy all of its dependencies have a total size of less than 20MB.
  - A docker image with rclpy based on `python:3.12-alpine` only takes 263 MB, compared with 875MB for `ros:jazzy-ros-base`
- **Portable**: Allows ROS to be run on different Linux distros. The only requirement is `x86_64`.

## Example Usage

Below steps will install and run the ROS Python client for the latest stable distribution, currently Kilted.

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

See [WHEEL_BUILDING.md](./WHEEL_BUILDING.md)
