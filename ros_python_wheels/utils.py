import subprocess


def get_system_python_version() -> str:
    """Get the system Python version from /usr/bin/python3."""
    result = subprocess.run(
        [
            "/usr/bin/python3",
            "-c",
            "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    version = result.stdout.strip()
    print(f"Detected system Python version: {version}")
    return version
