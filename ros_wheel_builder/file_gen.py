import os
from pathlib import Path

from google import genai
from jinja2 import Environment, FileSystemLoader


def generate_description_with_gemini(readme_content, package_name) -> str | None:
    """
    Generate a short description using Google Gemini API.

    Args:
        readme_content: The full README content
        package_name: The name of the package

    Returns:
        str or None: Generated description or None if failed
    """
    client = genai.Client()

    prompt = f"""
Based on the following README content for a ROS (Robot Operating System) package named "{package_name}", 
generate a concise, plain text, single-sentence description (maximum 150 characters) that would be suitable for a Python package description field.

The description should:
- Be clear and informative
- Mention it's a ROS package
- Briefly explain the main functionality
- Not include installation instructions or technical details
- Be professional and suitable for package managers like PyPI

README content:
{readme_content[:2000]}  # Limit content to avoid token limits

Generate only the description text, no additional formatting or explanation.
"""

    response = client.models.generate_content(model="gemini-2.5-flash", contents=prompt)
    return response.text


DEFAULT_DESCRIPTION = "Wheel for the {package_name} ROS 2 package."

DEFAULT_LONG_DESCRIPTION = "Wheel for the {package_name} ROS 2 package. Built using https://github.com/ycheng517/ros-python-wheels"


def get_package_description(package_name: str, package_dir: str) -> tuple[str, str]:
    """
    Extract description and long_description from README.md files only.

    Returns:
        tuple: (description, long_description, long_description_content_type)
    """
    package_path = Path(package_dir)
    readme_path = package_path / "README.md"

    if not readme_path.exists():
        pypi_package_name = f"ros-{package_name.replace('_', '-')}"
        desc = f"Wheel for the {package_name} ({pypi_package_name}) ROS 2 package."
        long_desc = f"Wheel for the {package_name} ({pypi_package_name}) ROS 2 package. Built using https://github.com/ycheng517/ros-python-wheels"
        return desc, long_desc

    readme_content = "Built using https://github.com/ycheng517/ros-python-wheels.\n\n" + readme_path.read_text()
    description = generate_description_with_gemini(readme_content, package_path.name)
    if not description:
        description = DEFAULT_DESCRIPTION.format(package_name=package_name)
    return description, readme_content


def get_meta_package_description(
    package_name: str, distro_name: str
) -> tuple[str, str]:
    """
    Extract description and long_description from README.md files only.

    Returns:
        tuple: (description, long_description, long_description_content_type)
    """
    pypi_package_name = f"ros-{package_name.replace('_', '-')}"
    desc = f"Meta-package for {package_name} ({pypi_package_name}) for ROS 2 {distro_name}."
    long_desc = f"Meta-package for {package_name} ({pypi_package_name}) for ROS 2 {distro_name}. Built using https://github.com/ycheng517/ros-python-wheels"
    return desc, long_desc


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
    package_name,
    version,
    library_name,
    run_dependencies,
    extras_require=None,
    description=None,
    long_description=None,
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
        description=description,
        long_description=long_description,
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
    package_name,
    version,
    dependency_name,
    dependency_version,
    extras_require=None,
    description=None,
    long_description=None,
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
        description=description,
        long_description=long_description,
    )
