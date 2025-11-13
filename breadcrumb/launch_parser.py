"""Launch file parser for extracting static information."""

import importlib.util
from pathlib import Path
import warnings

from ament_index_python.packages import get_package_share_directory
from clingwrap.static_info import StaticInformation


class LaunchFileLoadError(Exception):
    """Raised when a launch file cannot be loaded."""

    pass


def get_launch_file_static_information(launch_file_path: Path) -> StaticInformation | None:
    """
    Load a ROS 2 launch file and extract its static information.

    This function dynamically loads a Python launch file, calls its
    generate_launch_description() function, and extracts static analysis
    information from the clingwrap LaunchBuilder.

    Args:
        launch_file_path: Path to the .launch.py file

    Returns:
        StaticInformation object from clingwrap containing nodes,
        containers, relays, and included launch files

    Raises:
        LaunchFileLoadError: If the launch file cannot be loaded or
                            is not a valid clingwrap launch file

    Example:
        >>> from pathlib import Path
        >>> static_info = load_launch_file(Path("my_robot.launch.py"))
        >>> print(f"Found {len(static_info.nodes)} nodes")
    """
    if not launch_file_path.exists():
        raise LaunchFileLoadError(f"Launch file not found: {launch_file_path}")

    if not launch_file_path.suffix == ".py":
        warnings.warn(
            f"Launch file is not a .py file: {launch_file_path}. Skipping.",
            UserWarning,
        )
        return None

    # Load the launch file as a Python module
    try:
        spec = importlib.util.spec_from_file_location("breadcrumb_launch", launch_file_path)
        if spec is None or spec.loader is None:
            raise LaunchFileLoadError(f"Could not create module spec for {launch_file_path}")

        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

    except LaunchFileLoadError:
        # Re-raise our own exceptions
        raise
    except Exception as e:
        raise LaunchFileLoadError(f"Failed to import {launch_file_path}: {e}") from e

    # Check for generate_launch_description function
    if not hasattr(module, "generate_launch_description"):
        raise LaunchFileLoadError(
            f"{launch_file_path.name} does not have a generate_launch_description() function. "
            "This is required for all ROS 2 launch files."
        )

    # Call generate_launch_description()
    try:
        launch_description = module.generate_launch_description()
    except Exception as e:
        raise LaunchFileLoadError(f"Error calling generate_launch_description(): {e}") from e

    # Verify it's a clingwrap LaunchBuilder
    if not hasattr(launch_description, "get_static_information"):
        warnings.warn(
            f"{launch_file_path.name} does not return a clingwrap LaunchBuilder. "
            "Breadcrumb requires launch files built with clingwrap for static analysis. "
            "Regular launch files using LaunchDescription cannot be analyzed statically. Skipping.",
            UserWarning,
        )
        return None

    # Extract and return static information
    try:
        return launch_description.get_static_information()
    except Exception as e:
        raise LaunchFileLoadError(f"Error extracting static information: {e}") from e


def get_launch_file_static_information_recursive(
    launch_file_path: Path, visited: set[Path] | None = None
) -> StaticInformation | None:
    if visited is None:
        visited = set()

    # Prevent circular includes
    resolved_path = launch_file_path.resolve()
    if resolved_path in visited:
        return None
    visited.add(resolved_path)

    # Get base static info
    static_info = get_launch_file_static_information(launch_file_path)

    if static_info is None:
        return None

    # Recursively process each included launch file
    launch_files_to_iterate = static_info.included_launch_files.copy()
    for include in launch_files_to_iterate:
        include_path = Path(get_package_share_directory(include.package)) / include.directory / include.launch_file

        if include.namespace is not None:
            warnings.warn(
                f"Launch file {include_path.name} included by {launch_file_path} with namespace '{include.namespace}'. "
                "This is currently not supported by breadcrumb and will be skipped.",
                UserWarning,
            )
            continue

        included_static_info = get_launch_file_static_information_recursive(include_path, visited)
        if included_static_info is not None:
            static_info.merge(included_static_info)
            static_info.included_launch_files.remove(include)

    return static_info
