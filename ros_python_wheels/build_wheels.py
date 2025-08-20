import os
import shutil
import sys
import subprocess
from pathlib import Path
from elftools.elf.elffile import ELFFile
from collections import deque
from elftools.common.exceptions import ELFError


IGNORE_LIST = [
    "ld-linux-x86-64.so",
    "libpython3.",
    "libstdc++.so",
    "libc.so",
    "libm.so",
    "libgcc_s.so",
    "libcrypto.so",
    "libssl.so",
    "libz.so",
    "linux-vdso.so",
]


def get_all_needed_libs(path: str) -> list[str]:
    """
    Recursively finds all shared library dependencies for a given ELF file.
    """
    # Standard system library paths
    search_paths = [
        "/lib",
        "/lib64",
        "/usr/lib",
        "/usr/lib64",
        "/lib/x86_64-linux-gnu",
        "/usr/lib/x86_64-linux-gnu",
    ]

    # Add paths from the LD_LIBRARY_PATH environment variable if it exists
    if "LD_LIBRARY_PATH" in os.environ:
        search_paths.extend(os.environ["LD_LIBRARY_PATH"].split(":"))

    # Add the directory of the initial library to the search paths
    path = os.path.abspath(path)
    initial_dir = os.path.dirname(os.path.abspath(path))
    if initial_dir not in search_paths:
        search_paths.insert(0, initial_dir)

    # A queue of libraries whose dependencies we need to find
    libs_to_process = deque([path])
    # A set to store the full paths of libraries we've already processed
    processed_paths = set()
    # A set to store the names of all found dependencies
    all_needed_libs = set()

    def find_library(name: str) -> str | None:
        """Helper function to find the full path of a library."""
        for p in search_paths:
            full_path = os.path.join(p, name)
            if os.path.exists(full_path):
                return full_path
        return None

    while libs_to_process:
        current_path = libs_to_process.popleft()

        if not os.path.isabs(current_path):
            # Try to resolve relative path or just name
            resolved_path = find_library(current_path)
            if not resolved_path:
                # print(f"Warning: Could not find library: {current_path}")
                continue
            current_path = resolved_path

        if current_path in processed_paths:
            continue

        processed_paths.add(current_path)

        try:
            with open(current_path, "rb") as f:
                elf = ELFFile(f)
                dynamic = elf.get_section_by_name(".dynamic")
                if not dynamic:
                    continue

                # Iterate through the DT_NEEDED tags
                for tag in dynamic.iter_tags():
                    if tag.entry.d_tag == "DT_NEEDED":
                        needed_lib = tag.needed
                        if (
                            needed_lib not in all_needed_libs
                            and needed_lib not in IGNORE_LIST
                        ):
                            all_needed_libs.add(needed_lib)
                            libs_to_process.append(needed_lib)
        except (ELFError, FileNotFoundError, IsADirectoryError) as e:
            # print(f"Warning: Could not process {current_path}: {e}")
            continue
    return sorted(list(all_needed_libs))


def find_system_library_path(so_file: str, needed_lib: str) -> str | None:
    """Find the system path for a needed library using ldd."""
    result = subprocess.run(["ldd", so_file], capture_output=True, text=True)

    for line in result.stdout.split("\n"):
        if needed_lib not in line or "=>" not in line:
            continue

        parts = line.split("=>")
        if len(parts) <= 1:
            continue

        lib_path = parts[1].split("(")[0].strip()
        if lib_path and lib_path != "not found" and os.path.exists(lib_path):
            return lib_path

    return None


def copy_dependency(lib_path: str, so_file: str, needed_lib: str) -> None:
    """Copy a dependency to the same directory as the .so file."""
    dest_dir = os.path.dirname(so_file)
    dest_path = os.path.join(dest_dir, needed_lib)
    print(f"  Copying {lib_path} to {dest_path}")
    shutil.copy2(lib_path, dest_path)


def build_wheel(build_dir: str, output_dir: str) -> bool:
    """Run the pip wheel command to build the wheel."""
    try:
        cmd = [
            sys.executable,
            "-m",
            "pip",
            "wheel",
            "--no-build-isolation",
            "--no-deps",  # Don't build dependencies - they should come from PyPI
            "--wheel-dir",
            os.path.abspath(output_dir),
            build_dir,
        ]
        print(f"Running: {' '.join(cmd)}")

        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        print("Wheel built successfully!")
        print(f"Output: {result.stdout}")
        if result.stderr:
            print(f"Warnings: {result.stderr}")
        return True

    except subprocess.CalledProcessError as e:
        print(f"Error building wheel: {e}")
        print(f"stdout: {e.stdout}")
        print(f"stderr: {e.stderr}")
        return False


def do_build_wheels(build_dir: str, output_dir: str) -> None:
    """For all subdirectories in build_dir, build wheels."""
    build_path = Path(build_dir)
    output_path = Path(output_dir)

    if not build_path.exists():
        raise FileNotFoundError(f"Build directory '{build_dir}' does not exist.")

    if not output_path.exists():
        output_path.mkdir(parents=True, exist_ok=True)

    for subdir in build_path.iterdir():
        if subdir.is_dir():
            print(f"Building wheel for {subdir}")
            success = build_wheel(str(subdir), str(output_path))
            if not success:
                raise RuntimeError(f"Failed to build wheel for {subdir}")


def add_system_shared_libs(build_dir: str) -> None:
    build_path = Path(build_dir)

    # Step 1: Walk the build_dir and collect all .so and .so.* files
    so_files = []
    so_basenames = set()

    for root, dirs, files in os.walk(build_path):
        for file in files:
            if file.endswith(".so") or ".so." in file:
                full_path = os.path.join(root, file)
                so_files.append(full_path)
                so_basenames.add(file)

    print(f"Found {len(so_files)} shared library files in {build_dir}")

    # Step 2: For each .so file, check its dependencies
    for so_file in so_files:
        print(f"Processing {so_file}")
        needed_libs = get_all_needed_libs(so_file)
        print("need: ", needed_libs)

        for needed_lib in needed_libs:
            # Skip if dependency is already in our set
            if needed_lib in so_basenames:
                continue

            # Skip if dependency is in the ignore list
            if any(needed_lib.startswith(prefix) for prefix in IGNORE_LIST):
                print(f"  Skipping ignored dependency: {needed_lib}")
                continue

            print(f"  Need to find system dependency: {needed_lib}")
            lib_path = find_system_library_path(so_file, needed_lib)
            if not lib_path:
                raise RuntimeError(
                    f"System dependency '{needed_lib}' not found for {so_file}"
                )

            copy_dependency(lib_path, so_file, needed_lib)
            # so_basenames.add(needed_lib)


def build_wheels(build_dir: str = "build", output_dir: str = "dist") -> None:
    build_path = Path(build_dir)
    output_path = Path(output_dir)

    if not build_path.exists():
        raise FileNotFoundError(f"Build directory '{build_dir}' does not exist.")

    if not output_path.exists():
        output_path.mkdir(parents=True, exist_ok=True)

    # Step 1: Add system shared libraries to the build directory
    add_system_shared_libs(build_dir)

    # Step 2: Build the wheels
    do_build_wheels(build_dir, output_dir)
