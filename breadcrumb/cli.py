"""Command-line interface for breadcrumb."""

from pathlib import Path
import sys

import click

from .graph_builder import build_graph
from .launch_parser import LaunchFileLoadError, load_launch_file


@click.command()
@click.argument(
    "launch_file",
    type=click.Path(exists=True, dir_okay=False, path_type=Path),
)
def main(launch_file: Path):
    """Static analysis tool for ROS 2 node graphs.

    Analyzes a clingwrap launch file and displays the discovered nodes,
    topics, services, and actions.

    Examples:

      breadcrumb my_robot.launch.py

      breadcrumb /path/to/launch/file.launch.py
    """
    # Load the launch file
    try:
        static_info = load_launch_file(launch_file)
    except LaunchFileLoadError as e:
        click.echo(f"Error: {e}", err=True)
        sys.exit(1)

    # Build the graph
    graph = build_graph(static_info)

    # Print summary
    click.echo(f"ROS Graph from: {launch_file.name}")
    click.echo()

    # Print nodes
    click.echo(f"Nodes ({len(graph.nodes)}):")
    if graph.nodes:
        for node in graph.nodes:
            composable_marker = " [composable]" if node.is_composable else ""
            click.echo(f"  {node.id} ({node.package}){composable_marker}")
    else:
        click.echo("  (none)")
    click.echo()

    # Print placeholders for Phase 2
    click.echo(f"Topics ({len(graph.topics)}): (Phase 2)")
    click.echo(f"Services ({len(graph.services)}): (Phase 2)")
    click.echo(f"Actions ({len(graph.actions)}): (Phase 2)")

    # Print included launch files info
    if static_info.included_launch_files:
        click.echo()
        click.echo(f"Included Launch Files ({len(static_info.included_launch_files)}):")
        for include in static_info.included_launch_files:
            namespace_info = f" in namespace: {include.namespace}" if include.namespace else ""
            click.echo(f"  {include.package}/{include.directory}/{include.launch_file}{namespace_info}")

    # Print topic relays info
    if static_info.topic_relays:
        click.echo()
        click.echo(f"Topic Relays ({len(static_info.topic_relays)}):")
        for relay in static_info.topic_relays:
            relay_info = f"{relay.relay_type}"
            if relay.rate:
                relay_info += f" @ {relay.rate}Hz"
            click.echo(f"  {relay.from_topic} -> {relay.to_topic} ({relay_info})")
