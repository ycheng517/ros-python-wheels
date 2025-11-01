#!/bin/bash
set -e
echo "--- Running custom wheel repair script ---"
WHEEL_FILE="$1"
DEST_DIR="$2"

ENV_PREFIX=$(python -c 'import sys; print(sys.prefix)')
echo "Found env prefix at: $ENV_PREFIX"
SITE_PACKAGES=$(python -c 'import site; print(site.getsitepackages()[0])')
echo "Found site-packages at: $SITE_PACKAGES"

EXCLUDE_OPTS=""
# Initialize FINAL_LD_PATH with the existing LD_LIBRARY_PATH, or empty if unset
FINAL_LD_PATH="${LD_LIBRARY_PATH:-}:$ENV_PREFIX/lib"
# Exclude libraries from the venv lib directory
if [ -d "$ENV_PREFIX/lib" ]; then
    echo "Excluding libraries from $ENV_PREFIX/lib"
    for lib in $(find "$ENV_PREFIX/lib" -maxdepth 1 -name "*.so*"); do
        EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
    done
fi

if [ -d "build" ]; then
    PYTHON_ID_PART=$(python -c 'import sys; print(f"{sys.implementation.name}-{sys.version_info.major}{sys.version_info.minor}")')
    BUILD_TEMP_DIR=$(find build -maxdepth 1 -type d -name "temp.*-${PYTHON_ID_PART}" | head -n 1)
    if [ -n "$BUILD_TEMP_DIR" ] && [ -d "$BUILD_TEMP_DIR" ]; then
        # Add the cmake build directory
        CMAKE_BUILD_DIR="$(pwd)/$BUILD_TEMP_DIR/build"
        if [ -d "$CMAKE_BUILD_DIR" ]; then
            echo "Found temp cmake build dir: $CMAKE_BUILD_DIR, adding to LD_LIBRARY_PATH"
            FINAL_LD_PATH="$FINAL_LD_PATH:$CMAKE_BUILD_DIR"
        fi

        # Add the cmake install directory
        INSTALL_LIB_DIR="$(pwd)/$BUILD_TEMP_DIR/install/lib"
        if [ -d "$INSTALL_LIB_DIR" ]; then
            echo "Found temp install lib dir: $INSTALL_LIB_DIR, adding to LD_LIBRARY_PATH"
            FINAL_LD_PATH="$FINAL_LD_PATH:$INSTALL_LIB_DIR"
            for lib in $(find "$INSTALL_LIB_DIR" -name "*.so*"); do
                EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
            done
        fi

        # Add the cmake install lib64 directory (for packages that install to lib64)
        INSTALL_LIB64_DIR="$(pwd)/$BUILD_TEMP_DIR/install/lib64"
        if [ -d "$INSTALL_LIB64_DIR" ]; then
            echo "Found temp install lib64 dir: $INSTALL_LIB64_DIR, adding to LD_LIBRARY_PATH"
            FINAL_LD_PATH="$FINAL_LD_PATH:$INSTALL_LIB64_DIR"
            for lib in $(find "$INSTALL_LIB64_DIR" -name "*.so*"); do
                EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
            done
        fi
    fi
fi

echo "Searching for ROS libraries in site-packages..."
for pkg_dir in "$SITE_PACKAGES"/ros_*; do
    if [ -d "$pkg_dir" ] && [[ $pkg_dir == *.libs ]]; then
        echo "Found ROS lib dir: $pkg_dir, adding to LD_LIBRARY_PATH and excluding its libraries"
        FINAL_LD_PATH="$FINAL_LD_PATH:$pkg_dir"
        # Find all .so files in the directory and add them to the exclude list
        for lib in $(find "$pkg_dir" -name "*.so*"); do
            EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
        done
    fi
done

echo "Searching for ROS libraries in lib/x86_64-linux-gnu/"
if [ -d "$ENV_PREFIX/lib/x86_64-linux-gnu" ]; then
    FINAL_LD_PATH="$FINAL_LD_PATH:$ENV_PREFIX/lib/x86_64-linux-gnu"
    for lib in $(find "$ENV_PREFIX/lib/x86_64-linux-gnu" -maxdepth 1 -name "*.so*"); do
        echo "Excluding library $(basename $lib) from lib/x86_64-linux-gnu/"
        EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
    done
fi

echo "Searching for ROS libraries in opt/zenoh_cpp_vendor/lib64/"
if [ -d "$ENV_PREFIX/opt/zenoh_cpp_vendor/lib64" ]; then
    FINAL_LD_PATH="$FINAL_LD_PATH:$ENV_PREFIX/opt/zenoh_cpp_vendor/lib64"
    for lib in $(find "$ENV_PREFIX/opt/zenoh_cpp_vendor/lib64" -maxdepth 1 -name "*.so*"); do
        echo "Excluding library $(basename $lib) from opt/zenoh_cpp_vendor/lib64/"
        EXCLUDE_OPTS="$EXCLUDE_OPTS --exclude $(basename $lib)"
    done
fi

# Use 'export' or 'env' to set the variable for the auditwheel command
export LD_LIBRARY_PATH="$FINAL_LD_PATH"
echo "Final LD_LIBRARY_PATH: $LD_LIBRARY_PATH"
auditwheel repair --only-plat --plat manylinux_2_28_x86_64 $EXCLUDE_OPTS -w "$DEST_DIR" "$WHEEL_FILE"

echo "--- Custom wheel repair script finished ---"