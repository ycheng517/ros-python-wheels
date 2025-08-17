import fire
import os
import shutil
import sys
import subprocess
from pathlib import Path
from elftools.elf.elffile import ELFFile


IGNORE_LIST = ["ld-linux-x86-64", "libpython3"]


def get_needed_libs(path: str):
    with open(path, "rb") as f:
        elf = ELFFile(f)
        dynamic = elf.get_section_by_name(".dynamic")
        if not dynamic:
            return []
        return [
            tag.needed for tag in dynamic.iter_tags() if tag.entry.d_tag == "DT_NEEDED"
        ]


def find_system_library_path(so_file: str, needed_lib: str):
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


def copy_dependency(lib_path: str, so_file: str, needed_lib: str):
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


def main(build_dir: str):
    build_path = Path(build_dir)

    # Step 1: Walk the build_dir and collect all .so and .so.* files
    so_files = set()
    so_basenames = set()

    for root, dirs, files in os.walk(build_path):
        for file in files:
            if file.endswith(".so") or ".so." in file:
                full_path = os.path.join(root, file)
                so_files.add(full_path)
                so_basenames.add(file)

    print(f"Found {len(so_files)} shared library files in {build_dir}")

    # Step 2: For each .so file, check its dependencies
    for so_file in so_files:
        print(f"Processing {so_file}")
        needed_libs = get_needed_libs(so_file)

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
            so_basenames.add(needed_lib)


if __name__ == "__main__":
    fire.Fire(main)
