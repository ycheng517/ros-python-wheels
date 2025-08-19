#!/usr/bin/env python3
import fire

from ros_python_wheels import build_wheels, build_python_artifacts


def build(
    ros_packages: list[str],
    build_dir: str = "build",
    ros_distro: str = "jazzy",
    dist_dir: str = "dist",
) -> None:
    """Build ROS packages and their Python artifacts, then build wheels.
    Args:
        ros_packages (list[str]): List of ROS package names to build.
        build_dir (str): Directory to build artifacts (default: "build").
        ros_distro (str): ROS distribution to target (default: "jazzy").
        dist_dir (str): Directory to output built wheels (default: "dist").
    """
    print(f"Building ROS packages: {ros_packages}")
    for ros_package in ros_packages:
        build_python_artifacts(ros_package, build_dir, ros_distro)

    print(f"Building wheels in {build_dir} and outputting to {dist_dir}")
    build_wheels(build_dir, dist_dir)


if __name__ == "__main__":
    fire.Fire(build)
