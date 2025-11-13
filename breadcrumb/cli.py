"""Command-line interface for breadcrumb."""

from pathlib import Path
from pprint import pprint
import sys
import warnings

import click
from clingwrap.static_info import StaticInformation

from .launch_parser import LaunchFileLoadError, get_launch_file_static_information_recursive


@click.command()
@click.argument(
    "launch_files",
    nargs=-1,
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
def main(launch_files: tuple[Path, ...]):
    """Static analysis tool for ROS 2 node graphs.

    Analyzes one or more clingwrap launch files and displays the discovered
    nodes, topics, services, and actions in a unified graph.

    Examples:

      breadcrumb my_robot.launch.py

      breadcrumb robot.launch.py sensors.launch.py navigation.launch.py

      breadcrumb /path/to/*.launch.py
    """
    # Configure warnings to display nicely with click
    def warning_handler(message, category, filename, lineno, file=None, line=None):
        click.secho(f"Warning: {message}", fg="yellow", err=True)

    warnings.showwarning = warning_handler

    static_infos: list[tuple[Path, "StaticInformation"]] = []  # type: ignore
    for launch_file in launch_files:
        try:
            static_info = get_launch_file_static_information_recursive(launch_file)
            static_infos.append((launch_file, static_info))
        except LaunchFileLoadError as e:
            click.echo(f"Error loading {launch_file.name}: {e}", err=True)
            sys.exit(1)

    pprint(static_infos, depth=4)
