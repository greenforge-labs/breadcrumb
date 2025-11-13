"""Command-line interface for breadcrumb."""

from pathlib import Path
from pprint import pprint
import sys
import warnings

import click
from clingwrap.static_info import ComposableNodeInfo, NodeInfo, StaticInformation

from .helpers import LaunchFileSource, get_launch_file_sources_from_included_launch_files, get_package_name_from_path
from .launch_parser import LaunchFileLoadError, get_launch_file_static_information
from .node_interface import NodeInterface, load_node_interface


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

    static_infos: list[tuple[LaunchFileSource, StaticInformation]] = []

    launch_files_to_process: list[LaunchFileSource] = [LaunchFileSource(path=p) for p in launch_files]

    while len(launch_files_to_process) > 0:
        launch_file = launch_files_to_process.pop()

        if launch_file.package is None:
            launch_file.package = get_package_name_from_path(launch_file.path)
            if launch_file.package is None:
                warnings.warn(
                    f"Could not find a package name for {launch_file}.",
                    UserWarning,
                )

        try:
            static_info = get_launch_file_static_information(launch_file.path)
            if static_info is None:
                continue

            launch_files_to_process += get_launch_file_sources_from_included_launch_files(
                static_info.included_launch_files
            )

            static_infos.append((launch_file, static_info))

        except LaunchFileLoadError as e:
            click.echo(f"Error loading {launch_file.path.name}: {e}.", err=True)
            sys.exit(1)

    # pprint(static_infos, depth=4)

    all_nodes: list[tuple[NodeInfo | ComposableNodeInfo, NodeInterface | None, LaunchFileSource]] = []

    for launch_source, static_info in static_infos:
        nodes: list[NodeInfo | ComposableNodeInfo] = static_info.nodes + static_info.get_all_composable_nodes()

        for node in nodes:
            try:
                interface = load_node_interface(
                    node.package,
                    executable=node.executable if hasattr(node, "executable") else None,  # type: ignore
                    plugin=node.plugin if hasattr(node, "plugin") else None,  # type: ignore
                    launching_package=launch_source.package,
                )
            except FileNotFoundError as e:
                warnings.warn(str(e), UserWarning)
                interface = None

            all_nodes.append((node, interface, launch_source))

    # pprint(all_nodes, depth=4)
