"""Helper utilities for breadcrumb."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from clingwrap.static_info import LaunchFileInclude

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from breadcrumb.graph import Node


@dataclass
class LaunchFileSource:
    path: Path
    package: str | None = None


def get_launch_file_sources_from_included_launch_files(
    included_launch_files: list[LaunchFileInclude],
) -> list[LaunchFileSource]:
    sources = []
    for include in included_launch_files:
        sources.append(
            LaunchFileSource(
                path=Path(get_package_share_directory(include.package)) / include.directory / include.launch_file,
                package=include.package,
            )
        )
    return sources


def get_package_name_from_path(path: Path) -> str | None:
    """
    Determine the ROS2 package name by traversing up the path hierarchy
    until a package.xml file is found.

    Args:
        path: File or directory path to start searching from

    Returns:
        The package name from package.xml, or None if no package.xml is found

    Raises:
        ValueError: If package.xml is found but doesn't contain a valid name tag
    """
    current_path = path.resolve()

    # If it's a file, start from its parent directory
    if current_path.is_file():
        current_path = current_path.parent

    # Traverse up the directory hierarchy
    while current_path != current_path.parent:  # Stop at filesystem root
        package_xml = current_path / "package.xml"

        if package_xml.exists():
            try:
                tree = ET.parse(package_xml)
                root = tree.getroot()

                # Find the <name> tag
                name_element = root.find("name")
                if name_element is not None and name_element.text:
                    return name_element.text.strip()
                else:
                    raise ValueError(f"package.xml found at {package_xml} but does not contain a valid <name> tag")
            except ET.ParseError as e:
                raise ValueError(f"Failed to parse package.xml at {package_xml}: {e}")

        # Move up one directory level
        current_path = current_path.parent

    return None


def extract_group_name(namespace: str | None) -> str | None:
    """
    Extract the first element of a namespace for grouping purposes.

    Args:
        namespace: The node namespace (e.g., "robot1/sensors/camera" or "robot1")

    Returns:
        The first namespace element (e.g., "robot1"), or None if namespace is None/empty

    Examples:
        >>> extract_group_name("robot1/sensors/camera")
        'robot1'
        >>> extract_group_name("robot1")
        'robot1'
        >>> extract_group_name(None)
        None
        >>> extract_group_name("")
        None
    """
    if not namespace:
        return None

    # Strip leading/trailing slashes and split on '/'
    cleaned = namespace.strip("/")
    if not cleaned:
        return None

    # Get the first element
    parts = cleaned.split("/")
    return parts[0]


def group_nodes_by_namespace(nodes: list["Node"]) -> dict[str | None, list["Node"]]:
    """
    Group nodes by the first element of their namespace.

    Args:
        nodes: List of nodes to group

    Returns:
        Dictionary mapping group names to lists of nodes.
        Nodes with no namespace are mapped to None.

    Examples:
        If nodes have namespaces ["robot1/sensors", "robot1/control", "robot2/sensors", None]:
        Returns: {"robot1": [...], "robot2": [...], None: [...]}
    """
    groups: dict[str | None, list["Node"]] = {}

    for node in nodes:
        group_name = extract_group_name(node.namespace)
        if group_name not in groups:
            groups[group_name] = []
        groups[group_name].append(node)

    return groups
