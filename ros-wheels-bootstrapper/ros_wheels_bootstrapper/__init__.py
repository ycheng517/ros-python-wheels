import os
import sys
import site
from pathlib import Path

def bootstrap():
    """
    Sets up the environment for ROS wheels by modifying LD_LIBRARY_PATH.
    This logic is based on the shell script in `examples/pub_sub/setup.sh`.
    """
    site_packages = ""
    # In Python, we can get the site-packages directory more reliably.
    try:
        # getsitepackages() returns a list of paths. The first one is usually the one we want.
        site_packages_list = site.getsitepackages()
        if site_packages_list:
            site_packages = site_packages_list[0]
    except (AttributeError):
        pass # getsitepackages might not exist in some environments.

    if not site_packages:
        # Fallback for some environments, e.g. venv without --system-site-packages
        if sys.platform == "win32":
            site_packages = os.path.join(sys.prefix, 'Lib', 'site-packages')
        else:
            py_version = f'{sys.version_info.major}.{sys.version_info.minor}'
            site_packages = os.path.join(sys.prefix, 'lib', f'python{py_version}', 'site-packages')

    # Get existing LD_LIBRARY_PATH, or an empty string. Split it into a list.
    # Filter out empty strings that can result from a leading/trailing/double colon.
    ld_path_parts = [part for part in os.environ.get('LD_LIBRARY_PATH', '').split(':') if part]

    # Add `sys.prefix/lib` to the path, similar to `$ENV_PREFIX/lib` in the script.
    prefix_lib = os.path.join(sys.prefix, 'lib')
    if prefix_lib not in ld_path_parts:
        ld_path_parts.append(prefix_lib)

    # Search for ROS library directories in site-packages.
    # The shell script iterates through `ros_*` and checks if it ends with `.libs`.
    # This is equivalent to globbing for `ros_*.libs`.
    site_packages_path = Path(site_packages)
    if site_packages_path.is_dir():
        for libs_dir in site_packages_path.glob('ros_*.libs'):
            if libs_dir.is_dir():
                path_str = str(libs_dir.resolve())
                if path_str not in ld_path_parts:
                    ld_path_parts.append(path_str)

    # Join the parts back together and set the environment variable for the current process.
    os.environ['LD_LIBRARY_PATH'] = ':'.join(ld_path_parts)

# A simple guard to ensure this bootstrap logic runs only once per process.
if not os.environ.get('_ROS_WHEELS_BOOTSTRAP_COMPLETE'):
    bootstrap()
    os.environ['_ROS_WHEELS_BOOTSTRAP_COMPLETE'] = '1'
