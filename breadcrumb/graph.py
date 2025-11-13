"""Graph data structures and denormalization functions for breadcrumb."""

from dataclasses import dataclass, field
from pathlib import Path

from clingwrap.static_info import ComposableNodeInfo, NodeInfo

from .helpers import LaunchFileSource
from .node_interface import NodeInterface

from typing import Any


@dataclass
class Node:
    """A ROS2 node in the graph."""

    fqn: str  # Fully qualified name (e.g., "/namespace/node_name")
    name: str  # Node name without namespace
    namespace: str | None  # Namespace (e.g., "robot1/sensors")
    package: str
    executable: str | None = None  # For regular nodes
    plugin: str | None = None  # For composable nodes
    node_type: str = "regular"  # "regular" or "composable"
    source_launch_file: Path | None = None
    parameters: dict[str, Any] = field(default_factory=dict)


@dataclass
class Topic:
    """A ROS2 topic in the graph."""

    name: str  # Fully qualified topic name
    msg_type: str  # Message type (e.g., "std_msgs/msg/String")
    publishers: list[Node] = field(default_factory=list)
    subscribers: list[Node] = field(default_factory=list)


@dataclass
class Service:
    """A ROS2 service in the graph."""

    name: str  # Fully qualified service name
    srv_type: str  # Service type (e.g., "std_srvs/srv/SetBool")
    providers: list[Node] = field(default_factory=list)
    clients: list[Node] = field(default_factory=list)


@dataclass
class Action:
    """A ROS2 action in the graph."""

    name: str  # Fully qualified action name
    action_type: str  # Action type (e.g., "nav2_msgs/action/NavigateToPose")
    servers: list[Node] = field(default_factory=list)
    clients: list[Node] = field(default_factory=list)


@dataclass
class Graph:
    """Complete ROS2 graph representation."""

    nodes: list[Node] = field(default_factory=list)
    topics: list[Topic] = field(default_factory=list)
    services: list[Service] = field(default_factory=list)
    actions: list[Action] = field(default_factory=list)


def resolve_node_fqn(node_name: str | None, namespace: str | None) -> tuple[str, str]:
    """
    Resolve fully qualified node name.

    Args:
        node_name: Node name (may be None)
        namespace: Namespace string (e.g., "robot1/sensors")

    Returns:
        Tuple of (fqn, resolved_name) where fqn includes namespace and resolved_name is the bare name
    """
    # If no node name, return placeholder
    if node_name is None:
        resolved_name = "unnamed_node"
    else:
        resolved_name = node_name

    # Build FQN
    if namespace:
        # Ensure namespace doesn't have leading/trailing slashes
        clean_namespace = namespace.strip("/")
        fqn = f"/{clean_namespace}/{resolved_name}"
    else:
        fqn = f"/{resolved_name}"

    return fqn, resolved_name


def resolve_name(
    name: str,
    node_fqn: str,
    namespace: str | None,
) -> str:
    """
    Apply ROS2 name resolution rules.

    Args:
        name: Topic/service/action name (may be global, private, or relative)
        node_fqn: Fully qualified node name (e.g., "/namespace/node_name")
        namespace: Namespace string (e.g., "robot1/sensors")

    Returns:
        Fully resolved name

    ROS2 name resolution rules:
    - Global (starts with "/"): use as-is
    - Private (starts with "~"): resolve to node_fqn/name_without_tilde
    - Relative: resolve to /namespace/name
    """
    # Convert to string if it's a Substitution object
    name_str = str(name)

    # Global name
    if name_str.startswith("/"):
        return name_str

    # Private name (node-relative)
    if name_str.startswith("~"):
        # Remove the tilde and any leading slash, then append to node FQN
        private_name = name_str[1:].lstrip("/")
        return f"{node_fqn}/{private_name}"

    # Relative name (namespace-relative)
    if namespace:
        clean_namespace = namespace.strip("/")
        return f"/{clean_namespace}/{name_str}"
    else:
        return f"/{name_str}"


def apply_remappings(name: str, remappings: dict[Any, Any] | None) -> str:
    """
    Apply remapping rules to a name.

    Args:
        name: The name to potentially remap
        remappings: Dictionary of source -> target remappings (may contain Substitution objects)

    Returns:
        Remapped name if a match is found, otherwise original name
    """
    if remappings is None:
        return name

    # Check for exact match in remappings
    # Note: remappings may contain Substitution objects, convert to strings
    for source, target in remappings.items():
        source_str = str(source)
        target_str = str(target)

        if name == source_str:
            return target_str

    return name


def build_graph(
    all_nodes: list[tuple[NodeInfo | ComposableNodeInfo, NodeInterface | None, LaunchFileSource]],
) -> Graph:
    """
    Denormalize all_nodes into a unified graph structure.

    Args:
        all_nodes: List of tuples containing (node_info, interface, launch_source)
                   interface may be None for nodes without interface files

    Returns:
        Graph object containing all nodes, topics, services, and actions
    """
    graph = Graph()

    # Dictionaries for aggregating topics/services/actions
    topics_dict: dict[str, Topic] = {}
    services_dict: dict[str, Service] = {}
    actions_dict: dict[str, Action] = {}

    # First pass: Create all Node objects
    for node_info, interface, launch_source in all_nodes:
        # Extract node metadata
        node_name = node_info.name
        namespace = node_info.namespace
        package = node_info.package

        # Determine node type and executable/plugin
        if isinstance(node_info, ComposableNodeInfo):
            node_type = "composable"
            executable = None
            plugin = node_info.plugin
        else:
            node_type = "regular"
            executable = node_info.executable
            plugin = None

        # If node_name is None, try to use executable or plugin as fallback
        if node_name is None:
            node_name = executable or plugin or "unnamed_node"

        # Resolve fully qualified name
        fqn, resolved_name = resolve_node_fqn(node_name, namespace)

        # Extract parameters (convert any Substitution objects to strings)
        parameters = {}
        if node_info.parameters:
            for key, value in node_info.parameters.items():
                parameters[str(key)] = str(value) if value is not None else None

        # Create Node object
        node = Node(
            fqn=fqn,
            name=resolved_name,
            namespace=namespace,
            package=package,
            executable=executable,
            plugin=plugin,
            node_type=node_type,
            source_launch_file=launch_source.path,
            parameters=parameters,
        )

        graph.nodes.append(node)

        # Second pass: Process interface if available
        if interface is None:
            # Node without interface - skip connection processing
            continue

        remappings = node_info.remappings

        # Process publishers
        for pub in interface.publishers:
            # Resolve topic name
            resolved_topic = resolve_name(pub.topic, fqn, namespace)
            # Apply remappings
            final_topic = apply_remappings(resolved_topic, remappings)

            # Create or update Topic object
            if final_topic not in topics_dict:
                topics_dict[final_topic] = Topic(name=final_topic, msg_type=pub.type)
            topics_dict[final_topic].publishers.append(node)

        # Process subscribers
        for sub in interface.subscribers:
            # Resolve topic name
            resolved_topic = resolve_name(sub.topic, fqn, namespace)
            # Apply remappings
            final_topic = apply_remappings(resolved_topic, remappings)

            # Create or update Topic object
            if final_topic not in topics_dict:
                topics_dict[final_topic] = Topic(name=final_topic, msg_type=sub.type)
            topics_dict[final_topic].subscribers.append(node)

        # Process services (providers)
        for svc in interface.services:
            # Resolve service name
            resolved_service = resolve_name(svc.name, fqn, namespace)
            # Apply remappings
            final_service = apply_remappings(resolved_service, remappings)

            # Create or update Service object
            if final_service not in services_dict:
                services_dict[final_service] = Service(name=final_service, srv_type=svc.type)
            services_dict[final_service].providers.append(node)

        # Process service clients
        for svc_client in interface.service_clients:
            # Resolve service name
            resolved_service = resolve_name(svc_client.name, fqn, namespace)
            # Apply remappings
            final_service = apply_remappings(resolved_service, remappings)

            # Create or update Service object
            if final_service not in services_dict:
                services_dict[final_service] = Service(name=final_service, srv_type=svc_client.type)
            services_dict[final_service].clients.append(node)

        # Process actions (servers)
        for act in interface.actions:
            # Resolve action name
            resolved_action = resolve_name(act.name, fqn, namespace)
            # Apply remappings
            final_action = apply_remappings(resolved_action, remappings)

            # Create or update Action object
            if final_action not in actions_dict:
                actions_dict[final_action] = Action(name=final_action, action_type=act.type)
            actions_dict[final_action].servers.append(node)

        # Process action clients
        for act_client in interface.action_clients:
            # Resolve action name
            resolved_action = resolve_name(act_client.name, fqn, namespace)
            # Apply remappings
            final_action = apply_remappings(resolved_action, remappings)

            # Create or update Action object
            if final_action not in actions_dict:
                actions_dict[final_action] = Action(name=final_action, action_type=act_client.type)
            actions_dict[final_action].clients.append(node)

    # Convert dictionaries to lists
    graph.topics = list(topics_dict.values())
    graph.services = list(services_dict.values())
    graph.actions = list(actions_dict.values())

    return graph
