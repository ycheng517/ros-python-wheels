# ROS Python Wheels

This project helps convert ROS Python packages into Python wheels that can be distributed and installed via pip.

## Features

- Convert individual ROS Python packages to wheels
- **NEW**: Recursive building of all ROS Python dependencies
- Automatic dependency resolution using rosdep
- Support for shared libraries in ROS runtime dependencies
- Package metadata extraction from package.xml and egg-info

## Usage

### Basic Usage

Build a single ROS package:

```bash
python3 build_python_package.py sensor_msgs
```

### Recursive Building

Build a ROS package and all its ROS Python dependencies:

```bash
python3 build_python_package.py --recursive sensor_msgs
```

This will:

1. Analyze the dependency tree of `sensor_msgs`
2. Identify all ROS Python packages it depends on
3. Build each dependency in the correct order
4. Finally build the target package
5. Skip packages that already have wheels built

### Command Line Options

- `--recursive, -r`: Build all ROS Python dependencies recursively
- `--ros-distro`: Specify ROS distribution (default: jazzy)
- `--output-dir`: Output directory for wheels (default: dist)
- `--verbose, -v`: Enable verbose output

### Examples

```bash
# Build with all dependencies
python3 build_python_package.py --recursive --verbose geometry_msgs

# Build for different ROS distro
python3 build_python_package.py --recursive --ros-distro humble sensor_msgs

# Custom output directory
python3 build_python_package.py --recursive --output-dir /tmp/wheels sensor_msgs
```

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

## Output

The tool creates standard Python wheels (`.whl` files) with:

- Proper package metadata
- All Python modules and packages
- Bundled shared libraries when needed
- Correct dependency declarations

These wheels can be installed with `pip install` and distributed via PyPI or private repositories.
