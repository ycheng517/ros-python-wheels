import os
import json
from pathlib import Path
import re

import fire
import subprocess


def get_uploaded_packages_file(repository: str) -> Path:
    """Get the path to the uploaded packages tracking file for a repository."""
    return Path(f".uploaded_packages_{repository}.json")


def parse_wheel_filename(filename: str) -> tuple[str, str]:
    """Parse package name and version from wheel filename.

    Args:
        filename: The wheel filename (e.g., "ros_action_msgs-2.0.3-cp310-cp310-manylinux_2_28_x86_64.whl")

    Returns:
        A tuple of (package_name, version)
    """
    # Remove .whl extension
    basename = filename.replace(".whl", "")

    # Use regex to parse wheel filename according to PEP 427
    # Format: {name}-{version}(-{build tag})?-{python tag}-{abi tag}-{platform tag}
    pattern = r"^([^-]+)-([^-]+)(?:-\d+[^-]*)?-[^-]+-[^-]+-[^-]+$"
    match = re.match(pattern, basename)

    if match:
        return match.group(1), match.group(2)
    else:
        # Fallback: split on first two dashes
        parts = basename.split("-")
        if len(parts) >= 2:
            return parts[0], parts[1]
        else:
            raise ValueError(f"Cannot parse wheel filename: {filename}")


def yank_gemfury_package(package_name: str, version: str) -> bool:
    """Yank a package from Gemfury.

    Args:
        package_name: The name of the package to yank (with underscores from wheel filename)
        version: The version of the package to yank

    Returns:
        True if successful, False otherwise
    """
    # Convert underscores to hyphens for Gemfury package names
    gemfury_package_name = package_name.replace("_", "-")
    command = ["fury", "yank", f"{gemfury_package_name}@{version}", "--force"]

    print(f"Yanking package from Gemfury: {gemfury_package_name}@{version}")
    try:
        subprocess.run(command, check=True, capture_output=True, text=True)
        print(f"Successfully yanked {gemfury_package_name}@{version}")
        return True
    except FileNotFoundError:
        print("Error: 'fury' command not found. Please install Gemfury CLI.")
        return False
    except subprocess.CalledProcessError as e:
        print(f"Failed to yank {gemfury_package_name}@{version}: {e}")
        if e.stderr:
            print(f"Error details: {e.stderr.strip()}")
        return False


def load_uploaded_packages(repository: str) -> set[str]:
    """Load the set of already uploaded package filenames for a repository."""
    uploaded_file = get_uploaded_packages_file(repository)
    if uploaded_file.exists():
        try:
            with open(uploaded_file, "r") as f:
                data = json.load(f)
                return set(data.get("uploaded", []))
        except (json.JSONDecodeError, FileNotFoundError):
            return set()
    return set()


def save_uploaded_package(repository: str, package_filename: str) -> None:
    """Mark a package as uploaded for a repository."""
    uploaded_file = get_uploaded_packages_file(repository)
    uploaded_packages = load_uploaded_packages(repository)
    uploaded_packages.add(package_filename)

    with open(uploaded_file, "w") as f:
        json.dump({"uploaded": sorted(list(uploaded_packages))}, f, indent=2)


def is_package_uploaded(repository: str, package_filename: str) -> bool:
    """Check if a package has already been uploaded to a repository."""
    uploaded_packages = load_uploaded_packages(repository)
    return package_filename in uploaded_packages


def upload_wheels(wheels_dir="build/artifacts", repository="testpypi"):
    """Upload wheels to PyPI or TestPyPI.

    Args:
        wheels_dir: The directory containing the wheels to upload.
        repository: The repository to upload to (cloudsmith, pypi or testpypi).
    """
    supported = ["cloudsmith", "pypi", "testpypi", "gemfury"]
    if repository not in supported:
        print(f"Error: repository must be one of {supported}, got '{repository}'")
        return

    if not os.path.isdir(wheels_dir):
        print(f"Error: Directory not found: {wheels_dir}")
        return

    dist_files = [
        os.path.join(wheels_dir, f)
        for f in os.listdir(wheels_dir)
        if f.endswith(".whl")
    ]

    if not dist_files:
        print(f"No wheels found in {wheels_dir}")
        return

    uploaded_count = 0
    skipped_count = 0

    for dist_file in dist_files:
        package_filename = os.path.basename(dist_file)

        # Check if this package has already been uploaded
        if is_package_uploaded(repository, package_filename):
            print(f"Skipping already uploaded package: {package_filename}")
            skipped_count += 1
            continue

        # upload each wheel individually with sleep to avoid 429 errors
        command = [
            "twine",
            "upload",
            "--disable-progress-bar",
            "--repository",
            repository,
            dist_file,
        ]

        # Only add --skip-existing for repositories that support it
        if repository in ["pypi", "testpypi"]:
            command.insert(-1, "--skip-existing")

        print(f"Running command: {' '.join(command)}")
        try:
            subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Successfully uploaded: {package_filename}")
            # Mark package as uploaded
            save_uploaded_package(repository, package_filename)
            uploaded_count += 1
        except subprocess.CalledProcessError as e:
            # Check if the error contains "409" (conflict/duplicate) - continue uploading
            stderr_output = e.stderr if e.stderr else ""
            stdout_output = e.stdout if e.stdout else ""

            # Check for 409 conflict in stderr, stdout, or error message
            if (
                "409" in stderr_output
                or "409" in stdout_output
                or "409 Conflict" in str(e)
            ):
                print(f"409 conflict error for {package_filename} (likely duplicate)")
                if stderr_output:
                    print(f"Error details: {stderr_output.strip()}")

                # For Gemfury, yank the package and retry
                if repository == "gemfury":
                    try:
                        package_name, version = parse_wheel_filename(package_filename)
                        print("Attempting to yank and retry for Gemfury...")

                        if yank_gemfury_package(package_name, version):
                            # Retry the upload
                            print(f"Retrying upload of {package_filename}...")
                            subprocess.run(
                                command, check=True, capture_output=True, text=True
                            )
                            print(
                                f"Successfully uploaded after yank: {package_filename}"
                            )
                            save_uploaded_package(repository, package_filename)
                            uploaded_count += 1
                            continue
                        else:
                            print(
                                f"Failed to yank package, skipping {package_filename}"
                            )
                            continue
                    except Exception as yank_error:
                        stderr_output = e.stderr if e.stderr else ""
                        stdout_output = e.stdout if e.stdout else ""
                        print(f"Error during yank and retry process: {yank_error}")
                        if stderr_output:
                            print(f"Error details: {stderr_output.strip()}")
                        if stdout_output:
                            print(f"Output details: {stdout_output.strip()}")
                        continue
                else:
                    print("Continuing with next package...")
                    # Mark as uploaded since it likely already exists for other repositories
                    save_uploaded_package(repository, package_filename)
                    uploaded_count += 1
                    continue
            else:
                print(f"Failed to upload {package_filename}: {e}")
                if stderr_output:
                    print(f"Error details: {stderr_output.strip()}")
                if stdout_output:
                    print(f"Output details: {stdout_output.strip()}")
                print("Stopping upload process on first failure.")
                return

    print("\nUpload summary:")
    print(f"  Uploaded: {uploaded_count}")
    print(f"  Skipped (already uploaded): {skipped_count}")
    print(f"  Total packages: {len(dist_files)}")


def list_uploaded_packages(repository: str):
    """List all packages that have been marked as uploaded for a repository."""
    uploaded_packages = load_uploaded_packages(repository)
    if not uploaded_packages:
        print(f"No packages have been marked as uploaded for repository '{repository}'")
        return

    print(f"Packages marked as uploaded for repository '{repository}':")
    for package in sorted(uploaded_packages):
        print(f"  {package}")
    print(f"\nTotal: {len(uploaded_packages)} packages")


def clear_uploaded_packages(repository: str, confirm: bool = False):
    """Clear the list of uploaded packages for a repository.

    Args:
        repository: The repository to clear
        confirm: If True, skip confirmation prompt
    """
    uploaded_file = get_uploaded_packages_file(repository)

    if not uploaded_file.exists():
        print(f"No uploaded packages file found for repository '{repository}'")
        return

    uploaded_packages = load_uploaded_packages(repository)
    if not uploaded_packages:
        print(f"No packages marked as uploaded for repository '{repository}'")
        return

    if not confirm:
        response = input(
            f"Are you sure you want to clear {len(uploaded_packages)} uploaded packages for '{repository}'? (y/N): "
        )
        if response.lower() != "y":
            print("Cancelled.")
            return

    uploaded_file.unlink()
    print(f"Cleared uploaded packages list for repository '{repository}'")


def remove_uploaded_package(repository, package_filename):
    """Remove a specific package from the uploaded packages list."""
    uploaded_packages = load_uploaded_packages(repository)

    if package_filename not in uploaded_packages:
        print(
            f"Package '{package_filename}' is not marked as uploaded for repository '{repository}'"
        )
        return

    uploaded_packages.remove(package_filename)

    uploaded_file = get_uploaded_packages_file(repository)
    with open(uploaded_file, "w") as f:
        json.dump({"uploaded": sorted(list(uploaded_packages))}, f, indent=2)

    print(
        f"Removed '{package_filename}' from uploaded packages list for repository '{repository}'"
    )


def main():
    """Main entry point that supports multiple commands."""
    fire.Fire(
        {
            "upload": upload_wheels,
            "list": list_uploaded_packages,
            "clear": clear_uploaded_packages,
            "remove": remove_uploaded_package,
        }
    )


if __name__ == "__main__":
    main()
