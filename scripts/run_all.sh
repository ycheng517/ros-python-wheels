#!/bin/bash

# Check if ROS distro argument is provided
if [ $# -eq 0 ]; then
    echo "Usage: $0 <ros_distro>"
    echo "Example: $0 kilted"
    exit 1
fi

ROS_DISTRO=$1

# Create unique run directory with timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
RUN_DIR="runs/run_${ROS_DISTRO}_${TIMESTAMP}"
CONTAINER_NAME="ros-python-wheels-${ROS_DISTRO}-${TIMESTAMP}"

echo "Creating run directory: ${RUN_DIR}"
mkdir -p "${RUN_DIR}"

docker build -f docker/build/${ROS_DISTRO}.dockerfile -t ros-python-wheels-${ROS_DISTRO} .

# Build the wheels
echo "Running container: ${CONTAINER_NAME}"
docker run --name ${CONTAINER_NAME} \
    --env CLOUDSMITH_USERNAME \
    --env CLOUDSMITH_API_KEY \
    ros-python-wheels-${ROS_DISTRO} \
    /ros_venv/bin/python scripts/run_all.py \
    --ros_distro=${ROS_DISTRO} --upload=True \
    common_interfaces,

# Copy build and dist directories from container
echo "Copying build/ and dist/ directories to ${RUN_DIR}"
docker cp ${CONTAINER_NAME}:/ros_python_wheels/build "${RUN_DIR}/" 2>/dev/null || echo "Warning: build/ directory not found in container"
docker cp ${CONTAINER_NAME}:/ros_python_wheels/dist "${RUN_DIR}/" 2>/dev/null || echo "Warning: dist/ directory not found in container"

# Clean up container
echo "Removing container: ${CONTAINER_NAME}"
docker rm ${CONTAINER_NAME}

echo "Artifacts saved to: ${RUN_DIR}"
