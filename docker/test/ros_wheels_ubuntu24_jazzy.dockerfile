FROM ubuntu:24.04

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    git \
    python3-pip \
    python3-venv

# Set up a virtual environment
RUN python3 -m venv ros_venv

# Install Python packages
RUN . /ros_venv/bin/activate && python3 -m pip install --extra-index-url https://dl.cloudsmith.io/public/ros-python-wheels/jazzy/python/simple/ \
    ros-rclpy[fastrtps] ros-test-msgs pytest

# copy the test script
COPY scripts/run_rclpy_tests.sh /run_rclpy_tests.sh

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
