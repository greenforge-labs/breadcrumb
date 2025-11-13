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
