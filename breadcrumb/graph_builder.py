"""Build ROS graph from static launch information."""

from clingwrap.static_info import ComposableNodeInfo, NodeInfo, StaticInformation

from .graph_types import Node, RosGraph


def _extract_node_name_from_plugin(plugin: str) -> str:
    """
    Extract node name from plugin string.

    Plugin format can be:
    - package::ClassName (e.g., "camera_package::CameraDriver")
    - package::namespace::ClassName (e.g., "cake_example::my_node::MyNode")

    We use the last component (ClassName) as the node name.

    Args:
        plugin: Plugin string

    Returns:
        Extracted class name (last component)
    """
    parts = plugin.split("::")
    if len(parts) >= 2:
        # Use the last component (ClassName)
        return parts[-1]
    # Fallback: use the whole plugin string
    return plugin


def _build_node_id(namespace: str | None, node_name: str) -> str:
    """
    Build fully qualified node ID from namespace and name.

    Args:
        namespace: Node namespace (e.g., "cleaning" or "robot1/sensors")
        node_name: Node name (e.g., "my_node")

    Returns:
        Fully qualified node ID (e.g., "/cleaning/my_node")
    """
    if namespace:
        # Ensure namespace doesn't start with /
        namespace = namespace.lstrip("/")
        return f"/{namespace}/{node_name}"
    else:
        return f"/{node_name}"


def _convert_regular_node(node_info: NodeInfo) -> Node:
    """
    Convert a clingwrap NodeInfo to a breadcrumb Node.

    Args:
        node_info: NodeInfo from clingwrap static analysis

    Returns:
        Node object for the graph
    """
    # Use specified name, or fall back to executable
    node_name = node_info.name if node_info.name else node_info.executable

    # Build fully qualified ID
    node_id = _build_node_id(node_info.namespace, node_name)

    # Convert remappings dict to simple string dict if present
    remappings = None
    if node_info.remappings:
        # Remappings are dict[SomeSubstitutionsType, SomeSubstitutionsType]
        # For Phase 1, we'll just convert to strings
        remappings = {str(k): str(v) for k, v in node_info.remappings.items()}

    return Node(
        id=node_id,
        package=node_info.package,
        executable=node_info.executable,
        name=node_name,
        namespace=node_info.namespace,
        is_composable=False,
        parameters=node_info.parameters,
        remappings=remappings,
    )


def _convert_composable_node(comp_node_info: ComposableNodeInfo, container_namespace: str | None) -> Node:
    """
    Convert a clingwrap ComposableNodeInfo to a breadcrumb Node.

    Args:
        comp_node_info: ComposableNodeInfo from clingwrap static analysis
        container_namespace: Namespace of the container (not automatically inherited)

    Returns:
        Node object for the graph with is_composable=True
    """
    # Extract node name from plugin if not specified
    if comp_node_info.name:
        node_name = comp_node_info.name
    else:
        node_name = _extract_node_name_from_plugin(comp_node_info.plugin)

    # Composable nodes use their own namespace (from namespace stack at creation)
    # NOT the container's namespace
    node_id = _build_node_id(comp_node_info.namespace, node_name)

    # Convert remappings dict to simple string dict if present
    remappings = None
    if comp_node_info.remappings:
        remappings = {str(k): str(v) for k, v in comp_node_info.remappings.items()}

    return Node(
        id=node_id,
        package=comp_node_info.package,
        executable=comp_node_info.plugin,
        name=node_name,
        namespace=comp_node_info.namespace,
        is_composable=True,
        parameters=comp_node_info.parameters,
        remappings=remappings,
    )


def build_graph(static_info: StaticInformation) -> RosGraph:
    """
    Build a ROS graph from clingwrap static information.

    Phase 1: Only builds the node list. Topics, services, and actions
    will be populated in Phase 2 when cake interfaces are loaded.

    Args:
        static_info: StaticInformation from clingwrap launch file analysis

    Returns:
        RosGraph with nodes populated, topics/services/actions empty
    """
    nodes: list[Node] = []

    # Convert regular nodes
    for node_info in static_info.nodes:
        nodes.append(_convert_regular_node(node_info))

    # Convert composable nodes from containers
    for container in static_info.composable_node_containers:
        for comp_node in container.nodes:
            nodes.append(_convert_composable_node(comp_node, container.namespace))

    # Create and return graph
    # Topics, services, and actions are empty for Phase 1
    return RosGraph(
        nodes=nodes,
        topics=[],  # Phase 2
        services=[],  # Phase 2
        actions=[],  # Phase 2
    )
