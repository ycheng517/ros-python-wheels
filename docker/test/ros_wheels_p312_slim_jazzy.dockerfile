FROM python:3.12-slim

# Install dependencies
RUN apt update && apt install -y \
    git

# Install Python packages
RUN python3 -m pip install --extra-index-url https://dl.cloudsmith.io/public/ros-python-wheels/jazzy/python/simple/ \
    ros-rclpy[fastrtps] ros-test-msgs pytest

# copy the test script
COPY scripts/run_rclpy_tests.sh /run_rclpy_tests.sh

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

CMD ["/bin/sh", "/run_rclpy_tests.sh"]