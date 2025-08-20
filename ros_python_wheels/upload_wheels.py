import json
import os
import hashlib
import requests
from pathlib import Path


def calculate_sha256(file_path: str) -> str:
    """Calculate the SHA256 hash of a file."""
    sha256_hash = hashlib.sha256()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            sha256_hash.update(chunk)
    return sha256_hash.hexdigest()


def upload_wheel_to_cloudsmith(
    wheel_path: str,
    username: str,
    api_key: str,
    repository: str = "ros-python-wheels",
    ros_distro: str = "jazzy",
) -> bool:
    """
    Upload a single wheel file to Cloudsmith using the 2-step process.

    Args:
        wheel_path: Path to the wheel file
        username: Cloudsmith username
        api_key: Cloudsmith API key
        repository: Repository name (default: "ros-python-wheels")
        ros_distro: ROS distribution (default: "jazzy")

    Returns:
        True if upload successful, False otherwise
    """
    if not os.path.exists(wheel_path):
        print(f"Error: Wheel file '{wheel_path}' does not exist.")
        return False

    wheel_filename = os.path.basename(wheel_path)

    # Calculate SHA256 hash
    sha256_hash = calculate_sha256(wheel_path)

    # Step 1: Upload the file and get an identifier
    upload_url = (
        f"https://upload.cloudsmith.io/{repository}/{ros_distro}/{wheel_filename}"
    )
    headers = {
        "Content-Sha256": sha256_hash,
    }

    # Prepare authentication
    auth = (username, api_key)

    print(f"Step 1: Uploading {wheel_filename} to get identifier...")

    try:
        with open(wheel_path, "rb") as f:
            response = requests.put(
                upload_url,
                data=f,
                headers=headers,
                auth=auth,
                timeout=300,  # 5 minute timeout
            )

        if response.status_code not in [200, 201]:
            print(f"Failed to upload {wheel_filename}. Status: {response.status_code}")
            print(f"Response: {response.text}")
            return False

        # Extract the identifier from the JSON response
        response_data = json.loads(response.text)
        identifier = response_data["identifier"]
        print(f"Step 1 successful. Got identifier: {identifier}")

        # Step 2: Create the package using the identifier
        create_url = f"https://api.cloudsmith.io/v1/packages/{repository}/{ros_distro}/upload/python/"
        create_headers = {
            "Content-Type": "application/json",
        }
        create_data = {"package_file": identifier}

        print(f"Step 2: Creating package for {wheel_filename}...")

        create_response = requests.post(
            create_url,
            json=create_data,
            headers=create_headers,
            auth=auth,
            timeout=300,
        )

        if create_response.status_code in [200, 201]:
            print(f"Successfully created package for {wheel_filename}")
            return True
        else:
            print(
                f"Failed to create package for {wheel_filename}. Status: {create_response.status_code}"
            )
            print(f"Response: {create_response.text}")
            return False

    except requests.exceptions.RequestException as e:
        print(f"Error uploading {wheel_filename}: {e}")
        return False


def upload_wheels_to_cloudsmith(
    folder_path: str = "dist",
    repository: str = "ros-python-wheels",
    ros_distro: str = "jazzy",
) -> None:
    """
    Upload all wheel files from a folder to Cloudsmith.
    Requires environment variables:
    - CLOUDSMITH_USERNAME
    - CLOUDSMITH_API_KEY

    Args:
        folder_path: Path to the folder containing wheel files
        repository: Repository name
        ros_distro: ROS distribution

    Returns:
        Tuple of (successful_uploads, total_wheels)
    """
    username = os.getenv("CLOUDSMITH_USERNAME")
    api_key = os.getenv("CLOUDSMITH_API_KEY")
    if not username or not api_key:
        raise ValueError(
            "Error: CLOUDSMITH_USERNAME and CLOUDSMITH_API_KEY environment variables must be set"
        )

    folder = Path(folder_path)
    if not folder.exists():
        raise FileNotFoundError(f"Folder '{folder_path}' does not exist.")

    if not folder.is_dir():
        raise ValueError(f"'{folder_path}' is not a directory.")

    # Find all wheel files
    wheel_files = list(folder.glob("*.whl"))

    if not wheel_files:
        raise FileNotFoundError(f"No wheel files found in '{folder_path}'")

    print(f"Found {len(wheel_files)} wheel files to upload:")
    for wheel_file in wheel_files:
        print(f"  - {wheel_file.name}")

    successful_uploads = 0

    for wheel_file in wheel_files:
        success = upload_wheel_to_cloudsmith(
            str(wheel_file), username, api_key, repository, ros_distro
        )

        if success:
            successful_uploads += 1

        print()  # Empty line for readability

    print(
        f"Upload complete: {successful_uploads}/{len(wheel_files)} wheels uploaded successfully"
    )
    if successful_uploads < len(wheel_files):
        raise RuntimeError(
            f"Some uploads failed: {successful_uploads}/{len(wheel_files)}"
        )
