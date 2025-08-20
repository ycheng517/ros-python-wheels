#!/bin/bash

# Check if ROS distro argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <ros_distro>"
    echo "Example: $0 kilted"
    exit 1
fi

ROS_DISTRO=$1

docker build -f docker/build/${ROS_DISTRO}.dockerfile -t ros-python-wheels-${ROS_DISTRO} .

# Build the wheels
docker run --rm \
    --env CLOUDSMITH_USERNAME \
    --env CLOUDSMITH_API_KEY \
    ros-python-wheels-${ROS_DISTRO} \
    /ros_venv/bin/python scripts/run_all.py \
    --ros_distro=${ROS_DISTRO} --upload=True \
    rmw_fastrtps_cpp,rclpy,test_msgs
