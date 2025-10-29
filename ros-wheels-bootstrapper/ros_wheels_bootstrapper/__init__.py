import os
import sys
import site
from pathlib import Path

def get_ros_paths():
    """
    Calculates the required paths, but does NOT set them.
    Returns a new LD_LIBRARY_PATH string.
    """
    site_packages = ""
    try:
        site_packages_list = site.getsitepackages()
        if site_packages_list:
            site_packages = site_packages_list[0]
    except AttributeError:
        pass  # Fallback below

    if not site_packages:
        # Fallback for some environments
        if sys.platform == "win32":
            site_packages = os.path.join(sys.prefix, 'Lib', 'site-packages')
        else:
            py_version = f'{sys.version_info.major}.{sys.version_info.minor}'
            site_packages = os.path.join(sys.prefix, 'lib', f'python{py_version}', 'site-packages')

    # Get existing LD_LIBRARY_PATH, or an empty string. Split it into a list.
    ld_path_parts = [part for part in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep) if part]
    
    # Use a set for efficient de-duplication
    ld_path_set = set(ld_path_parts)

    # Add `sys.prefix/lib` to the path
    prefix_lib = os.path.join(sys.prefix, 'lib')
    ld_path_set.add(prefix_lib)

    # Search for ROS library directories in site-packages
    site_packages_path = Path(site_packages)
    if site_packages_path.is_dir():
        for libs_dir in site_packages_path.glob('ros_*.libs'):
            if libs_dir.is_dir():
                ld_path_set.add(str(libs_dir.resolve()))

    # Re-join the parts. We put the existing paths at the end.
    new_parts = list(ld_path_set)
    # The original parts should be preserved in their order, but after the new ones.
    # A simple way is to just put our new finds first.
    final_parts = list(ld_path_set) # This set already includes the original parts
    
    # Note: On Windows, you'd modify 'PATH' instead of 'LD_LIBRARY_PATH'
    return os.pathsep.join(final_parts)


# --- This is the main logic that runs on import ---

# Check the guard variable
if not os.environ.get('_ROS_WHEELS_BOOTSTRAP_COMPLETE'):
    
    # 1. Calculate the new library path
    new_ld_path = get_ros_paths()
    
    # 2. Get a copy of the current environment
    new_env = os.environ.copy()
    
    # 3. Set the new variables for the *next* process
    new_env['LD_LIBRARY_PATH'] = new_ld_path
    new_env['_ROS_WHEELS_BOOTSTRAP_COMPLETE'] = '1'
    
    new_argv = [sys.executable] + sys.argv
    
    try:
        # sys.executable is the path to the python interpreter
        # new_argv is the list of arguments (e.g., ['/usr/bin/python3', 'my_script.py'])
        os.execve(sys.executable, new_argv, new_env)
        
    except Exception as e:
        sys.stderr.write(f"Failed to bootstrap ROS environment: {e}\n")
        sys.exit(1)

# If the guard variable *is* set, this script does nothing,
# and Python continues to your program (e.g., minimal_publisher.py)
# with the correct environment already set.

# Do not print here, as it will print *every* time,
# including after the successful re-exec.
# print(os.environ.get('LD_LIBRARY_PATH'))