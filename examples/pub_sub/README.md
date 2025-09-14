# ROS2 Python Wheels Example

This example demonstrates a minimal publisher and subscriber using `rclpy` and `std_msgs` installed from the ROS Python Wheels repository by [uv](https://github.com/astral-sh/uv).

## Installation

Navigate to this directory and run:

```bash
uv sync
```

## Usage

In each terminal, the following environment variables need to be set:

```bash
export LD_LIBRARY_PATH=$(uv pip show ros-rclpy | awk '/^Location:/ {print $2}')/ros_runtime_libs
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

1. **Run the subscriber:**

   Open a terminal and run:

   ```bash
   python minimal_subscriber.py
   ```

2. **Run the publisher:**

   Open another terminal and run:

   ```bash
   python minimal_publisher.py
   ```

You should see the subscriber receiving messages from the publisher.
