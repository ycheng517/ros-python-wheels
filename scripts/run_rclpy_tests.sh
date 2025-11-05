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
        RCLPY_VERSION="7.1.6"
        ;;
    "kilted")
        RCLPY_VERSION="9.1.2"
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

# Initialize counters
total_tests=0
failed_tests=0
allowed_failures=0

# List of tests that are allowed to fail
allowed_to_fail=("test_type_description_service.py" "test_destruction_order.py")

# List of tests to skip completely
skip_tests=("test_executor.py")

for f in rclpy/test/test_*.py; do
    test_name=$(basename "$f")

    # Skip tests in the skip list
    if [[ " ${skip_tests[@]} " =~ " ${test_name} " ]]; then
        echo "⏭️  SKIPPED: $f"
        continue
    fi

    echo "Running test file: $f"
    
    if pytest "$f"; then
        echo "✓ PASSED: $f"
    else
        # Check if this test is in the allowed failures list
        if [[ " ${allowed_to_fail[@]} " =~ " ${test_name} " ]]; then
            echo "⚠️  FAILED (ALLOWED): $f"
            ((allowed_failures++))
        else
            echo "✗ FAILED: $f"
            ((failed_tests++))
        fi
    fi
    ((total_tests++))
done

# Print summary
echo ""
echo "==================== TEST SUMMARY ===================="
echo "Total test files: $total_tests"
echo "Failed test files: $failed_tests"
echo "Allowed failures: $allowed_failures"
echo "Passed test files: $((total_tests - failed_tests - allowed_failures))"
if [ $failed_tests -eq 0 ]; then
    echo "🎉 All tests passed!"
    if [ $allowed_failures -gt 0 ]; then
        echo "Note: $allowed_failures test(s) failed but are allowed to fail"
    fi
else
    echo "⚠️  $failed_tests test file(s) failed"
fi
echo "======================================================="

# Return appropriate exit code at the very end
if [ $failed_tests -gt 0 ]; then
    exit 1
fi
