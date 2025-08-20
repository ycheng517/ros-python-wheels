FROM python:3.11-alpine

# Install dependencies
RUN apk update && apk add --no-cache \
    git

# Install Python packages
RUN python3 -m pip install --extra-index-url https://dl.cloudsmith.io/public/ros-python-wheels/jazzy/python/simple/ \
    ros-rclpy[fastrtps] pytest

# copy the test script
COPY scripts/run_rclpy_tests.sh /run_rclpy_tests.sh

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
