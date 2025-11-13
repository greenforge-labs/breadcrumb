"""Helper utilities for breadcrumb."""

from dataclasses import dataclass
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from clingwrap.static_info import LaunchFileInclude


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
