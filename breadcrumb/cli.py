"""Command-line interface for breadcrumb."""

import json
from pathlib import Path
import sys
import warnings

import click
from clingwrap.static_info import ComposableNodeInfo, NodeInfo, StaticInformation

from .graph import build_graph
from .helpers import LaunchFileSource, get_launch_file_sources_from_included_launch_files, get_package_name_from_path
from .launch_parser import LaunchFileLoadError, get_launch_file_static_information
from .node_interface import NodeInterface, load_node_interface
from .serialization import serialize_graph, serialize_to_dot


@click.command()
@click.argument(
    "launch_files",
    nargs=-1,
    required=True,
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
@click.option(
    "--output",
    "-o",
    type=click.Path(path_type=Path),
    help="Output file to write the graph. Format determined by extension: .json, .dot",
)
@click.option(
    "--include-hidden",
    is_flag=True,
    default=False,
    help="Include entities whose names start with underscore (hidden by default)",
)
def main(launch_files: tuple[Path, ...], output: Path | None, include_hidden: bool):
    """Static analysis tool for ROS 2 node graphs.

    Analyzes one or more clingwrap launch files and displays the discovered
    nodes, topics, services, and actions in a unified graph.

    Examples:

      breadcrumb my_robot.launch.py

      breadcrumb robot.launch.py sensors.launch.py navigation.launch.py

      breadcrumb /path/to/*.launch.py

      breadcrumb my_robot.launch.py -o graph.json

      breadcrumb my_robot.launch.py -o graph.dot
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

    # Build the denormalized graph
    graph = build_graph(all_nodes, include_hidden=include_hidden)

    # Display the graph
    click.echo("=" * 80)
    click.echo(f"Graph Analysis Complete")
    click.echo("=" * 80)
    click.echo(f"\nNodes: {len(graph.nodes)}")
    click.echo(f"Topics: {len(graph.topics)}")
    click.echo(f"Services: {len(graph.services)}")
    click.echo(f"Actions: {len(graph.actions)}")
    click.echo("\n" + "=" * 80)

    if output:
        # Detect format from file extension
        suffix = output.suffix.lower()

        if suffix == ".json":
            graph_data = serialize_graph(graph)
            with open(output, "w") as f:
                json.dump(graph_data, f, indent=2)
        elif suffix == ".dot":
            dot_content = serialize_to_dot(graph)
            with open(output, "w") as f:
                f.write(dot_content)
        else:
            raise click.BadParameter(
                f"Unsupported file extension: {suffix}. " f"Supported formats: .json, .dot, .mmd, .mermaid"
            )

        click.echo(f"Graph written to: {output}")
