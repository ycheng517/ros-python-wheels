# ROS Python Wheels

This project helps convert ROS Python packages into Python wheels that can be installed via pip.
A number of ROS Python wheels are uploaded to PyPi.

## Benefits

- **Easy Project Integration**: To include The ROS Python Client (rclpy) in a Python project, simply add `ros-rclpy[fastrtps]` to your `pyproject.toml`
- **Portable** Allows ROS to be run on different linux distros. The only requirement is x86_64.
- **Lightweight** rclpy all of its dependencies only takes up less than 25MB.

## Example: Install and Run the RCL Python Client

```bash
# install rclpy
pip install ros-rclpy[fastrtps]

# set LD_LIBRARY_PATH to the ros_runtime_libs directory of site-packages
# where ros-rclpy is installed
export LD_LIBRARY_PATH=$(pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs

# Run ROS
python -c "import rclpy; rclpy.init()"
```

## Building Wheels

### Basic Usage

Build a single ROS package to the `dist/` directory:

```bash
python3 build_python_package.py sensor_msgs
```

### Recursive Building

Build a ROS package and all its dependencies to the `dist/` directory:

```bash
python3 build_python_package.py --recursive sensor_msgs
```

This will:

1. Analyze the dependency tree of `sensor_msgs`
2. Identify all ROS packages it depends on
3. Build each dependency in the correct order
4. Finally build the target package
5. Skip packages that already have wheels built

## Supported Packages

## How Recursive Building Works

1. **Dependency Analysis**: Uses `rosdep` and `catkin_pkg` to analyze the dependency tree
2. **Filtering**: Only processes ROS packages that contain Python code
3. **Build Order**: Builds dependencies before dependents
4. **Caching**: Skips packages that already have wheels built
5. **Error Handling**: Stops on first build failure to prevent corrupted dependency chains

## Package Types Handled

- **ROS Python Packages**: Packages with Python code in `/opt/ros/<distro>/lib/python*/site-packages/`
- **ROS Runtime Packages**: Packages with shared libraries (`.so` files) - their libraries are bundled
- **Python Dependencies**: Standard Python packages (added to `install_requires`)
- **System Dependencies**: System packages (documented but not bundled)
