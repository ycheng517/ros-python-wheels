#!/bin/bash
# Script to set up LD_LIBRARY_PATH for wheel repair
# This script should be sourced, not executed directly

ENV_PREFIX=$(python -c 'import sys; print(sys.prefix)')
SITE_PACKAGES=$(python -c 'import site; print(site.getsitepackages()[0])')

# Initialize FINAL_LD_PATH with the existing LD_LIBRARY_PATH, or empty if unset
FINAL_LD_PATH="${LD_LIBRARY_PATH:-}:$ENV_PREFIX/lib"

# Find the temporary build directory and add its build and install/lib to the path
# This is where dependent libraries from this package are located before wheeling.
if [ -d "build" ]; then
    PYTHON_ID_PART=$(python -c 'import sys; print(f"{sys.implementation.name}-{sys.version_info.major}{sys.version_info.minor}")')
    BUILD_TEMP_DIR=$(find build -maxdepth 1 -type d -name "temp.*-${PYTHON_ID_PART}" | head -n 1)
    if [ -n "$BUILD_TEMP_DIR" ] && [ -d "$BUILD_TEMP_DIR" ]; then
        # Add the cmake build directory
        CMAKE_BUILD_DIR="$(pwd)/$BUILD_TEMP_DIR/build"
        if [ -d "$CMAKE_BUILD_DIR" ]; then
            FINAL_LD_PATH="$FINAL_LD_PATH:$CMAKE_BUILD_DIR"
        fi

        # Add the cmake install directory
        INSTALL_LIB_DIR="$(pwd)/$BUILD_TEMP_DIR/install/lib"
        if [ -d "$INSTALL_LIB_DIR" ]; then
            FINAL_LD_PATH="$FINAL_LD_PATH:$INSTALL_LIB_DIR"
        fi
    fi
fi

# Searching for ROS libraries in site-packages...
for pkg_dir in "$SITE_PACKAGES"/ros_*; do
    if [ -d "$pkg_dir" ] && [[ $pkg_dir == *.libs ]]; then
        FINAL_LD_PATH="$FINAL_LD_PATH:$pkg_dir"
    fi
done

# Export the final LD_LIBRARY_PATH
export LD_LIBRARY_PATH="$FINAL_LD_PATH"