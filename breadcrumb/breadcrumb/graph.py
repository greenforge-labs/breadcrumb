"""Graph data structures and denormalization functions for breadcrumb."""

from dataclasses import dataclass, field
from pathlib import Path

from clingwrap.static_info import ComposableNodeInfo, NodeInfo

from .helpers import LaunchFileSource
from .node_interface import (
    DurabilityPolicy,
    LivelinessPolicy,
    NodeInterface,
    ParameterDefinition,
    QoS,
    ReliabilityPolicy,
    _extract_param_name,
    _is_param_reference,
)

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
class TopicConnection:
    """A connection to a topic with QoS and compatibility info."""

    node: Node
    qos: QoS | None = None
    compatible: bool = True
    warnings: list[str] = field(default_factory=list)


@dataclass
class Topic:
    """A ROS2 topic in the graph."""

    name: str  # Fully qualified topic name
    msg_type: str  # Message type (e.g., "std_msgs/msg/String")
    publishers: list[TopicConnection] = field(default_factory=list)
    subscribers: list[TopicConnection] = field(default_factory=list)


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


def resolve_qos(
    raw_qos: QoS | None,
    node_parameters: dict[str, Any],
    interface_parameters: dict[str, ParameterDefinition],
) -> QoS | None:
    """
    Resolve parameter references in QoS settings.

    Parameter resolution order:
    1. Launch file parameters (node_parameters)
    2. Default values from interface.yaml (interface_parameters)

    Args:
        raw_qos: QoS object potentially containing ${param:name} references
        node_parameters: Parameters from the launch file
        interface_parameters: Parameter definitions from interface.yaml

    Returns:
        Resolved QoS object or None
    """
    if raw_qos is None:
        return None

    def resolve_value(value: Any) -> Any:
        """Resolve a single value that may be a parameter reference."""
        if not _is_param_reference(value):
            return value

        param_name = _extract_param_name(value)
        if param_name is None:
            return value

        # First check launch file parameters
        if param_name in node_parameters:
            resolved = node_parameters[param_name]
            # Convert string to appropriate type if needed
            if resolved is not None:
                return resolved

        # Fall back to interface parameter default
        if param_name in interface_parameters:
            return interface_parameters[param_name].default_value

        # If not found, return the original reference (unresolved)
        return value

    # Resolve history
    history = resolve_value(raw_qos.history)
    if isinstance(history, str) and history != "ALL" and not _is_param_reference(history):
        # Try to convert string to int if it's a number
        try:
            history = int(history)
        except (ValueError, TypeError):
            pass

    # Resolve reliability
    reliability = resolve_value(raw_qos.reliability)
    if isinstance(reliability, str) and not _is_param_reference(reliability):
        try:
            reliability = ReliabilityPolicy(reliability)
        except ValueError:
            pass

    # Resolve durability
    durability = resolve_value(raw_qos.durability) if raw_qos.durability is not None else None
    if isinstance(durability, str) and not _is_param_reference(durability):
        try:
            durability = DurabilityPolicy(durability)
        except ValueError:
            pass

    # Resolve deadline_ms
    deadline_ms = resolve_value(raw_qos.deadline_ms) if raw_qos.deadline_ms is not None else None
    if isinstance(deadline_ms, str) and not _is_param_reference(deadline_ms):
        try:
            deadline_ms = int(deadline_ms)
        except (ValueError, TypeError):
            pass

    # Resolve lifespan_ms
    lifespan_ms = resolve_value(raw_qos.lifespan_ms) if raw_qos.lifespan_ms is not None else None
    if isinstance(lifespan_ms, str) and not _is_param_reference(lifespan_ms):
        try:
            lifespan_ms = int(lifespan_ms)
        except (ValueError, TypeError):
            pass

    # Resolve liveliness
    liveliness = resolve_value(raw_qos.liveliness) if raw_qos.liveliness is not None else None
    if isinstance(liveliness, str) and not _is_param_reference(liveliness):
        try:
            liveliness = LivelinessPolicy(liveliness)
        except ValueError:
            pass

    # Resolve lease_duration_ms
    lease_duration_ms = resolve_value(raw_qos.lease_duration_ms) if raw_qos.lease_duration_ms is not None else None
    if isinstance(lease_duration_ms, str) and not _is_param_reference(lease_duration_ms):
        try:
            lease_duration_ms = int(lease_duration_ms)
        except (ValueError, TypeError):
            pass

    return QoS(
        history=history,
        reliability=reliability,
        durability=durability,
        deadline_ms=deadline_ms,
        lifespan_ms=lifespan_ms,
        liveliness=liveliness,
        lease_duration_ms=lease_duration_ms,
    )


def check_qos_compatibility(
    pub_qos: QoS | None,
    sub_qos: QoS | None,
) -> tuple[bool, list[str]]:
    """
    Check QoS compatibility between a publisher and subscriber.

    Only performs checks when both publisher and subscriber have QoS defined.
    If either has None QoS, returns compatible with no warnings.

    ROS2 Compatibility Rules:
    - Reliability: Publisher must be >= Subscriber (RELIABLE >= BEST_EFFORT)
    - Durability: Publisher must be >= Subscriber (TRANSIENT_LOCAL >= VOLATILE)
    - Deadline: Subscriber deadline >= Publisher deadline
    - Liveliness: Publisher must be >= Subscriber (MANUAL_BY_TOPIC >= AUTOMATIC)
    - Lease duration: Subscriber lease >= Publisher lease

    Args:
        pub_qos: Publisher QoS settings
        sub_qos: Subscriber QoS settings

    Returns:
        Tuple of (compatible: bool, warnings: list[str])
    """
    # Skip check if either side has no QoS defined
    if pub_qos is None or sub_qos is None:
        return (True, [])

    warnings: list[str] = []

    # Reliability check: RELIABLE > BEST_EFFORT
    # Incompatible: BEST_EFFORT pub -> RELIABLE sub
    if isinstance(pub_qos.reliability, ReliabilityPolicy) and isinstance(sub_qos.reliability, ReliabilityPolicy):
        if pub_qos.reliability == ReliabilityPolicy.BEST_EFFORT and sub_qos.reliability == ReliabilityPolicy.RELIABLE:
            warnings.append(f"Reliability mismatch: publisher is BEST_EFFORT but subscriber requires RELIABLE")

    # Durability check: TRANSIENT_LOCAL > VOLATILE
    # Incompatible: VOLATILE pub -> TRANSIENT_LOCAL sub
    if isinstance(pub_qos.durability, DurabilityPolicy) and isinstance(sub_qos.durability, DurabilityPolicy):
        if pub_qos.durability == DurabilityPolicy.VOLATILE and sub_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            warnings.append(f"Durability mismatch: publisher is VOLATILE but subscriber requires TRANSIENT_LOCAL")

    # Deadline check: Subscriber deadline >= Publisher deadline
    # Incompatible: Sub deadline < Pub deadline
    if isinstance(pub_qos.deadline_ms, int) and isinstance(sub_qos.deadline_ms, int):
        if sub_qos.deadline_ms < pub_qos.deadline_ms:
            warnings.append(
                f"Deadline mismatch: subscriber deadline ({sub_qos.deadline_ms}ms) "
                f"< publisher deadline ({pub_qos.deadline_ms}ms)"
            )

    # Liveliness check: MANUAL_BY_TOPIC > AUTOMATIC
    # Incompatible: AUTOMATIC pub -> MANUAL_BY_TOPIC sub
    if isinstance(pub_qos.liveliness, LivelinessPolicy) and isinstance(sub_qos.liveliness, LivelinessPolicy):
        if pub_qos.liveliness == LivelinessPolicy.AUTOMATIC and sub_qos.liveliness == LivelinessPolicy.MANUAL_BY_TOPIC:
            warnings.append(f"Liveliness mismatch: publisher is AUTOMATIC but subscriber requires MANUAL_BY_TOPIC")

    # Lease duration check: Subscriber lease >= Publisher lease
    # Incompatible: Sub lease < Pub lease
    if isinstance(pub_qos.lease_duration_ms, int) and isinstance(sub_qos.lease_duration_ms, int):
        if sub_qos.lease_duration_ms < pub_qos.lease_duration_ms:
            warnings.append(
                f"Lease duration mismatch: subscriber lease ({sub_qos.lease_duration_ms}ms) "
                f"< publisher lease ({pub_qos.lease_duration_ms}ms)"
            )

    compatible = len(warnings) == 0
    return (compatible, warnings)


def _is_hidden(name: str) -> bool:
    """
    Check if a name represents a hidden entity (starts with underscore).

    Args:
        name: Name or FQN to check

    Returns:
        True if the last component starts with underscore
    """
    last_component = name.split("/")[-1]
    return last_component.startswith("_")


def build_graph(
    all_nodes: list[tuple[NodeInfo | ComposableNodeInfo, NodeInterface | None, LaunchFileSource]],
    include_hidden: bool = False,
) -> Graph:
    """
    Denormalize all_nodes into a unified graph structure.

    Args:
        all_nodes: List of tuples containing (node_info, interface, launch_source)
                   interface may be None for nodes without interface files
        include_hidden: If False, filter out entities whose names start with underscore

    Returns:
        Graph object containing all nodes, topics, services, and actions
    """
    graph = Graph()

    # Dictionaries for aggregating topics/services/actions
    topics_dict: dict[str, Topic] = {}
    services_dict: dict[str, Service] = {}
    actions_dict: dict[str, Action] = {}

    # Track hidden nodes to filter them from connections
    hidden_nodes: set[str] = set()

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

        # Filter hidden nodes if not including them
        if not include_hidden and _is_hidden(resolved_name):
            hidden_nodes.add(fqn)
            continue

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
            # Apply remappings
            remapped_topic = apply_remappings(pub.topic, remappings)
            # Resolve topic name
            final_topic = resolve_name(remapped_topic, fqn, namespace)

            # Skip hidden topics if not including them
            if not include_hidden and _is_hidden(final_topic):
                continue

            # Resolve QoS parameters
            resolved_qos = resolve_qos(pub.qos, parameters, interface.parameters)

            # Create TopicConnection
            pub_conn = TopicConnection(node=node, qos=resolved_qos)

            # Create or update Topic object
            if final_topic not in topics_dict:
                topics_dict[final_topic] = Topic(name=final_topic, msg_type=pub.type)
            topics_dict[final_topic].publishers.append(pub_conn)

        # Process subscribers
        for sub in interface.subscribers:
            # Apply remappings
            remapped_topic = apply_remappings(sub.topic, remappings)
            # Resolve topic name
            final_topic = resolve_name(remapped_topic, fqn, namespace)

            # Skip hidden topics if not including them
            if not include_hidden and _is_hidden(final_topic):
                continue

            # Resolve QoS parameters
            resolved_qos = resolve_qos(sub.qos, parameters, interface.parameters)

            # Create TopicConnection
            sub_conn = TopicConnection(node=node, qos=resolved_qos)

            # Create or update Topic object
            if final_topic not in topics_dict:
                topics_dict[final_topic] = Topic(name=final_topic, msg_type=sub.type)
            topics_dict[final_topic].subscribers.append(sub_conn)

        # Process services (providers)
        for svc in interface.services:
            # Apply remappings
            remapped_service = apply_remappings(svc.name, remappings)
            # Resolve service name
            final_service = resolve_name(remapped_service, fqn, namespace)

            # Skip hidden services if not including them
            if not include_hidden and _is_hidden(final_service):
                continue

            # Create or update Service object
            if final_service not in services_dict:
                services_dict[final_service] = Service(name=final_service, srv_type=svc.type)
            services_dict[final_service].providers.append(node)

        # Process service clients
        for svc_client in interface.service_clients:
            # Apply remappings
            remapped_service = apply_remappings(svc_client.name, remappings)
            # Resolve service name
            final_service = resolve_name(remapped_service, fqn, namespace)

            # Skip hidden services if not including them
            if not include_hidden and _is_hidden(final_service):
                continue

            # Create or update Service object
            if final_service not in services_dict:
                services_dict[final_service] = Service(name=final_service, srv_type=svc_client.type)
            services_dict[final_service].clients.append(node)

        # Process actions (servers)
        for act in interface.actions:
            # Apply remappings
            remapped_action = apply_remappings(act.name, remappings)
            # Resolve action name
            final_action = resolve_name(remapped_action, fqn, namespace)

            # Skip hidden actions if not including them
            if not include_hidden and _is_hidden(final_action):
                continue

            # Create or update Action object
            if final_action not in actions_dict:
                actions_dict[final_action] = Action(name=final_action, action_type=act.type)
            actions_dict[final_action].servers.append(node)

        # Process action clients
        for act_client in interface.action_clients:
            # Apply remappings
            remapped_action = apply_remappings(act_client.name, remappings)
            # Resolve action name
            final_action = resolve_name(remapped_action, fqn, namespace)

            # Skip hidden actions if not including them
            if not include_hidden and _is_hidden(final_action):
                continue

            # Create or update Action object
            if final_action not in actions_dict:
                actions_dict[final_action] = Action(name=final_action, action_type=act_client.type)
            actions_dict[final_action].clients.append(node)

    # Convert dictionaries to lists
    graph.topics = list(topics_dict.values())
    graph.services = list(services_dict.values())
    graph.actions = list(actions_dict.values())

    # Check QoS compatibility for all pub/sub pairs on each topic
    for topic in graph.topics:
        for pub_conn in topic.publishers:
            for sub_conn in topic.subscribers:
                compatible, warnings = check_qos_compatibility(pub_conn.qos, sub_conn.qos)
                if not compatible:
                    # Mark both sides as having compatibility issues
                    pub_conn.compatible = False
                    pub_conn.warnings.extend([f"→ {sub_conn.node.fqn}: {w}" for w in warnings])
                    sub_conn.compatible = False
                    sub_conn.warnings.extend([f"← {pub_conn.node.fqn}: {w}" for w in warnings])

    return graph
