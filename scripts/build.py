#!/usr/bin/env python3
import fire

from ros_python_wheels import build_wheels, build_python_artifacts


def build(
    ros_packages: list[str], build_dir: str = "build", dist_dir: str = "dist"
) -> None:
    print(f"Building ROS packages: {ros_packages}")
    for ros_package in ros_packages:
        build_python_artifacts(ros_package, build_dir)

    print(f"Building wheels in {build_dir} and outputting to {dist_dir}")
    build_wheels(build_dir, dist_dir)


if __name__ == "__main__":
    fire.Fire(build)
