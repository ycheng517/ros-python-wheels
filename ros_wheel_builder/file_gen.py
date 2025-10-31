from jinja2 import Environment, FileSystemLoader
import os


def generate_cpp_pyproject_toml(before_build_command):
    """
    Generates a pyproject.toml file for a C++ package.
    """
    template_dir = os.path.join(os.path.dirname(__file__), "templates", "cpp_pkg")
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("pyproject.toml.j2")

    return template.render(
        build_dependencies=["setuptools", "wheel"],
        before_build_command=before_build_command,
    )


def generate_cpp_setup_py(
    package_name, version, library_name, run_dependencies, extras_require=None
):
    """
    Generates a setup.py file for a C++ package.
    """
    template_dir = os.path.join(os.path.dirname(__file__), "templates", "cpp_pkg")
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("setup.py.j2")

    # Convert package_name to ros-package format
    formatted_package_name = f"ros-{package_name.replace('_', '-')}"

    return template.render(
        package_name=formatted_package_name,
        original_package_name=package_name,
        version=version,
        library_name=library_name,
        run_dependencies=run_dependencies,
        extras_require=extras_require or {},
    )


def generate_dummy_c():
    """
    Generates a dummy.c file.
    """
    template_dir = os.path.join(os.path.dirname(__file__), "templates", "cpp_pkg")
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("dummy.c.j2")

    return template.render()


def generate_repair_script():
    """
    Returns the content of the repair_wheel.sh script.
    """
    script_path = os.path.join(
        os.path.dirname(__file__), "templates", "cpp_pkg", "repair_wheel.sh"
    )
    with open(script_path, "r") as f:
        return f.read()


def generate_meta_package_setup_py(
    package_name, version, dependency_name, dependency_version, extras_require=None
):
    """
    Generates a setup.py file for a meta package.
    """
    template_dir = os.path.join(os.path.dirname(__file__), "templates", "meta_pkg")
    env = Environment(loader=FileSystemLoader(template_dir))
    template = env.get_template("setup.py.j2")

    return template.render(
        package_name=package_name,
        version=version,
        dependency_name=dependency_name,
        dependency_version=dependency_version,
        extras_require=extras_require or {},
    )
