from toposort import toposort_flatten

from .distro import Distro, PackageSource


def generate_build_order(packages: list[str], distro: Distro) -> list[str]:
    """
    Generates a build order for a list of packages.
    """
    dependency_graph = {}
    for pkg in packages:
        deps = distro.get_build_depends(pkg) | distro.get_run_depends(pkg)
        dependency_graph[pkg] = set(
            [d.dep_name for d in deps if d.source == PackageSource.ROS]
        )
    return toposort_flatten(dependency_graph)
