#!/bin/bash

mkdir -p /tmp/rclpy_test
cd /tmp/rclpy_test

git clone --no-checkout https://github.com/ros2/rclpy.git
cd rclpy
git sparse-checkout init --cone
git sparse-checkout set rclpy/test
git checkout 7.1.4
rm rclpy/test/__init__.py
touch rclpy/test/__init__.py

export LD_LIBRARY_PATH=$(pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs

# Initialize counters
total_tests=0
failed_tests=0

for f in rclpy/test/test_*.py; do
    echo "Running test file: $f"
    if pytest "$f"; then
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
