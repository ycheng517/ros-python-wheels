# Building Wheels

## Pre-requisites

You need to have the ROS version that you'd like to build wheels for installed. For example,
to build wheels for ROS 2 Jazzy, you need to have ROS 2 Jazzy installed on your system.

## Building

Install this package and activate the environment using your favorite Python
package/environment manager, i.e.

```bash
uv sync
source .venv/bin/activate
```

Build a ROS package and all of its dependencies to the `dist/` directory

```bash
python scripts/build.py test_msgs
```

Can build a list of ROS packages as well:

```bash
python scripts/build.py rmw_fastrtps_cpp,rclpy,test_msgs
```

## Uploading

The following command uploads wheels in the `dist/` directory to the `ros-python-wheels` repository in [Cloudsmith](https://cloudsmith.com/). Cloudsmith credentials
are needed.

```bash
python scripts/upload.py
```
