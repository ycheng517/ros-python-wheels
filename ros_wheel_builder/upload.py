import fire
import os
import subprocess


def upload_wheels(wheels_dir="build/artifacts", repository="testpypi"):
    """Upload wheels to PyPI or TestPyPI.

    Args:
        wheels_dir: The directory containing the wheels to upload.
        repository: The repository to upload to (cloudsmith, pypi or testpypi).
    """
    if repository not in ["cloudsmith", "pypi", "testpypi"]:
        print(
            f"Error: repository must be 'cloudsmith', 'pypi' or 'testpypi', got '{repository}'"
        )
        return

    if not os.path.isdir(wheels_dir):
        print(f"Error: Directory not found: {wheels_dir}")
        return

    dist_files = [
        os.path.join(wheels_dir, f) for f in os.listdir(wheels_dir)
        if f.endswith(".whl")
    ]

    if not dist_files:
        print(f"No wheels found in {wheels_dir}")
        return

    command = [
        "twine",
        "upload",
        "--repository",
        repository,
    ] + dist_files

    print(f"Running command: {' '.join(command)}")
    subprocess.run(command, check=True)


if __name__ == "__main__":
    fire.Fire(upload_wheels)
