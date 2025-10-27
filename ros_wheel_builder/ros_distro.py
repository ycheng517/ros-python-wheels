import os
import os
import requests
import yaml
from pathlib import Path

ROS_DISTRO_URL = "https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml"


def get_ros_distro_index():
    """
    Downloads the rosdistro index file.
    """
    cache_dir = Path.home() / ".cache" / "ros-wheel-builder"
    cache_dir.mkdir(parents=True, exist_ok=True)
    index_path = cache_dir / "index-v4.yaml"

    if not index_path.exists():
        response = requests.get(ROS_DISTRO_URL)
        response.raise_for_status()
        with open(index_path, "w") as f:
            f.write(response.text)

    with open(index_path, "r") as f:
        return yaml.safe_load(f)


def get_distro_file(distro_name: str):
    """
    Gets the distribution file for a given ROS distro.
    """
    index = get_ros_distro_index()
    if distro_name not in index["distributions"]:
        raise ValueError(f"Distro {distro_name} not found in rosdistro index.")

    distro_info = index["distributions"][distro_name]
    if "distribution" not in distro_info:
        raise ValueError(f"Distro {distro_name} has no distribution file.")

    distro_url = (
        "https://raw.githubusercontent.com/ros/rosdistro/master/"
        + distro_info["distribution"][0]
    )  # TODO: handle multiple distribution files

    cache_dir = Path.home() / ".cache" / "ros-wheel-builder"
    cache_dir.mkdir(parents=True, exist_ok=True)
    distro_path = cache_dir / f"{distro_name}.yaml"

    if not distro_path.exists():
        response = requests.get(distro_url)
        response.raise_for_status()
        with open(distro_path, "w") as f:
            f.write(response.text)

    with open(distro_path, "r") as f:
        return yaml.safe_load(f)


def find_package(distro_data, package_name):
    """
    Finds a package in the distro data.
    """
    for repo_name, repo_data in distro_data["repositories"].items():
        if "release" in repo_data and "packages" in repo_data["release"]:
            if package_name in repo_data["release"]["packages"]:
                return repo_data
        elif repo_name == package_name:
            return repo_data
    return None


def download_source(package_name, package_info, build_dir):
    """
    Downloads the source code of a package.
    """
    release_info = package_info["release"]
    url = release_info["url"]
    version = release_info["version"]
    tag_format = release_info["tags"]["release"]
    tag = tag_format.format(package=package_name, version=version)

    repo_path = Path(build_dir) / package_name

    if repo_path.exists():
        print(f"Directory {repo_path} already exists, skipping clone.")
        return

    command = f"git clone --quiet --branch {tag} {url} {repo_path}"
    print(f"Executing: {command}")
    os.system(command)
