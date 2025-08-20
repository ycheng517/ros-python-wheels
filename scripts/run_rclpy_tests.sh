#!/bin/bash

# Check if ROS_DISTRO argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <ROS_DISTRO>"
    echo "Supported distros: humble, jazzy, kilted"
    exit 1
fi

ROS_DISTRO="$1"

# Map ROS distro to rclpy version
case "$ROS_DISTRO" in
    "humble")
        RCLPY_VERSION="3.3.17"
        ;;
    "jazzy")
        RCLPY_VERSION="7.1.4"
        ;;
    "kilted")
        RCLPY_VERSION="9.1.1"
        ;;
    *)
        echo "Error: Unsupported ROS distro '$ROS_DISTRO'"
        echo "Supported distros: humble, jazzy, kilted"
        exit 1
        ;;
esac

echo "Using ROS distro: $ROS_DISTRO"
echo "Using rclpy version: $RCLPY_VERSION"

mkdir -p /tmp/rclpy_test
cd /tmp/rclpy_test

git clone --no-checkout https://github.com/ros2/rclpy.git
cd rclpy
git sparse-checkout init --cone
git sparse-checkout set rclpy/test
git checkout "$RCLPY_VERSION"
rm rclpy/test/__init__.py
touch rclpy/test/__init__.py

export LD_LIBRARY_PATH=$(python3 -m pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs

# Initialize counters
total_tests=0
failed_tests=0

for f in rclpy/test/test_*.py; do
    echo "Running test file: $f"
    if python3 -m pytest "$f"; then
        echo "✓ PASSED: $f"
    else
        echo "✗ FAILED: $f"
        ((failed_tests++))
    fi
    ((total_tests++))
done

# Print summary
echo ""
echo "==================== TEST SUMMARY ===================="
echo "Total test files: $total_tests"
echo "Failed test files: $failed_tests"
echo "Passed test files: $((total_tests - failed_tests))"
if [ $failed_tests -eq 0 ]; then
    echo "🎉 All tests passed!"
else
    echo "⚠️  $failed_tests test file(s) failed"
fi
echo "======================================================="

# Return appropriate exit code at the very end
if [ $failed_tests -gt 0 ]; then
    exit 1
fi
