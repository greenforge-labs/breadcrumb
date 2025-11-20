"""Serialization functions for converting graph objects to various formats."""

from pathlib import Path

from .graph import Action, Graph, Node, Service, Topic
from .helpers import extract_group_name, group_nodes_by_namespace

from typing import Any


def serialize_node(node: Node) -> dict[str, Any]:
    """
    Serialize a Node object to a JSON-serializable dictionary.

    Args:
        node: Node object to serialize

    Returns:
        Dictionary representation of the node
    """
    return {
        "fqn": node.fqn,
        "name": node.name,
        "namespace": node.namespace,
        "package": node.package,
        "executable": node.executable,
        "plugin": node.plugin,
        "node_type": node.node_type,
        "source_launch_file": str(node.source_launch_file) if node.source_launch_file else None,
        "parameters": node.parameters,
    }


def serialize_topic(topic: Topic) -> dict[str, Any]:
    """
    Serialize a Topic object to a JSON-serializable dictionary.

    Args:
        topic: Topic object to serialize

    Returns:
        Dictionary with node references as FQN strings
    """
    return {
        "name": topic.name,
        "msg_type": topic.msg_type,
        "publishers": [node.fqn for node in topic.publishers],
        "subscribers": [node.fqn for node in topic.subscribers],
    }


def serialize_service(service: Service) -> dict[str, Any]:
    """
    Serialize a Service object to a JSON-serializable dictionary.

    Args:
        service: Service object to serialize

    Returns:
        Dictionary with node references as FQN strings
    """
    return {
        "name": service.name,
        "srv_type": service.srv_type,
        "providers": [node.fqn for node in service.providers],
        "clients": [node.fqn for node in service.clients],
    }


def serialize_action(action: Action) -> dict[str, Any]:
    """
    Serialize an Action object to a JSON-serializable dictionary.

    Args:
        action: Action object to serialize

    Returns:
        Dictionary with node references as FQN strings
    """
    return {
        "name": action.name,
        "action_type": action.action_type,
        "servers": [node.fqn for node in action.servers],
        "clients": [node.fqn for node in action.clients],
    }


def serialize_graph(graph: Graph) -> dict[str, Any]:
    """
    Serialize a Graph object to a JSON-serializable dictionary.

    Args:
        graph: Graph object to serialize

    Returns:
        Complete dictionary representation of the graph with nodes listed
        once and referenced by FQN in topics/services/actions
    """
    return {
        "nodes": [serialize_node(node) for node in graph.nodes],
        "topics": [serialize_topic(topic) for topic in graph.topics],
        "services": [serialize_service(service) for service in graph.services],
        "actions": [serialize_action(action) for action in graph.actions],
    }


def _escape_dot_label(text: str) -> str:
    """Escape special characters for DOT labels."""
    return text.replace('"', '\\"').replace("\n", "\\n")


def _topic_node_attrs(topic_id: str, label: str) -> str:
    """Generate DOT node attributes for a topic node."""
    escaped_label = _escape_dot_label(label)
    return f'"{topic_id}" [label="{escaped_label}", shape=ellipse, style=filled, fillcolor=lightgreen];'


def _service_node_attrs(svc_id: str, label: str) -> str:
    """Generate DOT node attributes for a service node."""
    escaped_label = _escape_dot_label(label)
    return f'"{svc_id}" [label="{escaped_label}", shape=diamond, style=filled, fillcolor=lightyellow];'


def _action_node_attrs(act_id: str, label: str) -> str:
    """Generate DOT node attributes for an action node."""
    escaped_label = _escape_dot_label(label)
    return f'"{act_id}" [label="{escaped_label}", shape=hexagon, style=filled, fillcolor=lightcoral];'


def _ros_node_attrs(node_id: str, label: str, fillcolor: str = "lightblue") -> str:
    """Generate DOT node attributes for a ROS node."""
    escaped_label = _escape_dot_label(label)
    return f'"{node_id}" [label="{escaped_label}", style=filled, fillcolor={fillcolor}];'


def _topic_pub_edge(pub_id: str, topic_id: str) -> str:
    """Generate DOT edge for publisher to topic."""
    return f'"{pub_id}" -> "{topic_id}" [label="pub"];'


def _topic_sub_edge(topic_id: str, sub_id: str) -> str:
    """Generate DOT edge for topic to subscriber."""
    return f'"{topic_id}" -> "{sub_id}" [label="sub", dir=back];'


def _service_provide_edge(prov_id: str, svc_id: str) -> str:
    """Generate DOT edge for provider to service."""
    return f'"{prov_id}" -> "{svc_id}" [label="provide", style=dashed];'


def _service_call_edge(client_id: str, svc_id: str) -> str:
    """Generate DOT edge for client to service."""
    return f'"{svc_id}" -> "{client_id}" [label="call", style=dashed, dir=back];'


def _action_serve_edge(srv_id: str, act_id: str) -> str:
    """Generate DOT edge for server to action."""
    return f'"{srv_id}" -> "{act_id}" [label="serve", style=dotted];'


def _action_call_edge(client_id: str, act_id: str) -> str:
    """Generate DOT edge for client to action."""
    return f'"{act_id}" -> "{client_id}" [label="call", style=dotted, dir=back];'


def serialize_to_dot(graph: Graph) -> str:
    """
    Serialize a Graph object to GraphViz DOT format.

    Args:
        graph: Graph object to serialize

    Returns:
        String containing DOT format graph representation
    """
    lines = ["digraph ROS2Graph {"]
    lines.append("  rankdir=LR;")
    lines.append("  node [shape=box];")
    lines.append("")

    # Add nodes
    lines.append("  // Nodes")
    for node in graph.nodes:
        node_id = _escape_dot_label(node.fqn)
        label = f"{node.fqn}\\n({node.package})"
        lines.append(f"  {_ros_node_attrs(node_id, label)}")

    lines.append("")

    # Add topics as separate nodes
    lines.append("  // Topics")
    for topic in graph.topics:
        topic_id = _escape_dot_label(topic.name)
        label = f"{topic.name}\\n[{topic.msg_type}]"
        lines.append(f"  {_topic_node_attrs(topic_id, label)}")

        # Connect publishers to topic
        for pub in topic.publishers:
            pub_id = _escape_dot_label(pub.fqn)
            lines.append(f"  {_topic_pub_edge(pub_id, topic_id)}")

        # Connect topic to subscribers
        for sub in topic.subscribers:
            sub_id = _escape_dot_label(sub.fqn)
            lines.append(f"  {_topic_sub_edge(topic_id, sub_id)}")

    lines.append("")

    # Add services
    lines.append("  // Services")
    for service in graph.services:
        svc_id = _escape_dot_label(service.name)
        label = f"{service.name}\\n[{service.srv_type}]"
        lines.append(f"  {_service_node_attrs(svc_id, label)}")

        # Connect providers to service
        for provider in service.providers:
            prov_id = _escape_dot_label(provider.fqn)
            lines.append(f"  {_service_provide_edge(prov_id, svc_id)}")

        # Connect service to clients
        for client in service.clients:
            client_id = _escape_dot_label(client.fqn)
            lines.append(f"  {_service_call_edge(client_id, svc_id)}")

    lines.append("")

    # Add actions
    lines.append("  // Actions")
    for action in graph.actions:
        act_id = _escape_dot_label(action.name)
        label = f"{action.name}\\n[{action.action_type}]"
        lines.append(f"  {_action_node_attrs(act_id, label)}")

        # Connect servers to action
        for server in action.servers:
            srv_id = _escape_dot_label(server.fqn)
            lines.append(f"  {_action_serve_edge(srv_id, act_id)}")

        # Connect action to clients
        for client in action.clients:
            client_id = _escape_dot_label(client.fqn)
            lines.append(f"  {_action_call_edge(client_id, act_id)}")

    lines.append("}")
    return "\n".join(lines)


def _get_inter_group_communication(
    graph: Graph, groups: dict[str | None, list[Node]]
) -> tuple[list[Topic], list[Service], list[Action]]:
    """
    Filter topics, services, and actions to only those connecting different groups or root nodes.

    A communication entity is inter-group if:
    - It has nodes from multiple different named groups, OR
    - It involves root namespace nodes

    Args:
        graph: The complete graph
        groups: Dictionary mapping group names to lists of nodes

    Returns:
        Tuple of (topics, services, actions) that connect between groups or to/from root nodes
    """
    # Create mapping from node FQN to group name for quick lookup
    node_to_group: dict[str, str | None] = {}
    for group_name, group_nodes in groups.items():
        for node in group_nodes:
            node_to_group[node.fqn] = group_name

    inter_group_topics = []
    for topic in graph.topics:
        # Get groups involved in this topic
        pub_groups = {node_to_group.get(pub.fqn) for pub in topic.publishers}
        sub_groups = {node_to_group.get(sub.fqn) for sub in topic.subscribers}
        all_groups = pub_groups | sub_groups

        # Inter-group if:
        # 1. Multiple non-root groups are involved, OR
        # 2. Root namespace node (None) is involved
        all_groups_non_root = all_groups - {None}
        has_root = None in all_groups
        multiple_groups = len(all_groups_non_root) > 1

        if has_root or multiple_groups:
            inter_group_topics.append(topic)

    inter_group_services = []
    for service in graph.services:
        prov_groups = {node_to_group.get(prov.fqn) for prov in service.providers}
        client_groups = {node_to_group.get(client.fqn) for client in service.clients}
        all_groups = prov_groups | client_groups

        all_groups_non_root = all_groups - {None}
        has_root = None in all_groups
        multiple_groups = len(all_groups_non_root) > 1

        if has_root or multiple_groups:
            inter_group_services.append(service)

    inter_group_actions = []
    for action in graph.actions:
        srv_groups = {node_to_group.get(srv.fqn) for srv in action.servers}
        client_groups = {node_to_group.get(client.fqn) for client in action.clients}
        all_groups = srv_groups | client_groups

        all_groups_non_root = all_groups - {None}
        has_root = None in all_groups
        multiple_groups = len(all_groups_non_root) > 1

        if has_root or multiple_groups:
            inter_group_actions.append(action)

    return inter_group_topics, inter_group_services, inter_group_actions


def _generate_toplevel_graph(graph: Graph, groups: dict[str | None, list[Node]]) -> str:
    """
    Generate a top-level DOT graph showing groups and inter-group connections.

    Args:
        graph: The complete graph
        groups: Dictionary mapping group names to lists of nodes

    Returns:
        DOT format string for the top-level graph
    """
    lines = ["digraph ROS2TopLevel {"]
    lines.append("  rankdir=LR;")
    lines.append("  node [shape=box];")
    lines.append("")

    # Add group nodes
    lines.append("  // Groups")
    for group_name, group_nodes in groups.items():
        if group_name is not None:
            group_id = _escape_dot_label(group_name)
            node_count = len(group_nodes)
            label = f"{group_name}\\n({node_count} node{'s' if node_count != 1 else ''})"
            lines.append(f"  {_ros_node_attrs(group_id, label)}")

    lines.append("")

    # Add root namespace nodes
    root_nodes = groups.get(None, [])
    if root_nodes:
        lines.append("  // Root Namespace Nodes")
        for node in root_nodes:
            node_id = _escape_dot_label(node.fqn)
            label = f"{node.fqn}\\n({node.package})"
            lines.append(f'  {_ros_node_attrs(node_id, label, "lightyellow")}')

        lines.append("")

    # Get inter-group communication entities
    inter_topics, inter_services, inter_actions = _get_inter_group_communication(graph, groups)

    # Add inter-group topics
    if inter_topics:
        lines.append("  // Inter-group Topics")
        for topic in inter_topics:
            topic_id = _escape_dot_label(topic.name)
            label = f"{topic.name}\\n[{topic.msg_type}]"
            lines.append(f"  {_topic_node_attrs(topic_id, label)}")

            # Connect group nodes and root nodes to topic
            for pub in topic.publishers:
                pub_group = extract_group_name(pub.namespace)
                if pub_group is not None:
                    # Publisher is in a named group
                    group_id = _escape_dot_label(pub_group)
                    lines.append(f"  {_topic_pub_edge(group_id, topic_id)}")
                else:
                    # Publisher is a root namespace node
                    node_id = _escape_dot_label(pub.fqn)
                    lines.append(f"  {_topic_pub_edge(node_id, topic_id)}")

            for sub in topic.subscribers:
                sub_group = extract_group_name(sub.namespace)
                if sub_group is not None:
                    # Subscriber is in a named group
                    group_id = _escape_dot_label(sub_group)
                    lines.append(f"  {_topic_sub_edge(topic_id, group_id)}")
                else:
                    # Subscriber is a root namespace node
                    node_id = _escape_dot_label(sub.fqn)
                    lines.append(f"  {_topic_sub_edge(topic_id, node_id)}")

        lines.append("")

    # Add inter-group services
    if inter_services:
        lines.append("  // Inter-group Services")
        for service in inter_services:
            svc_id = _escape_dot_label(service.name)
            label = f"{service.name}\\n[{service.srv_type}]"
            lines.append(f"  {_service_node_attrs(svc_id, label)}")

            for provider in service.providers:
                prov_group = extract_group_name(provider.namespace)
                if prov_group is not None:
                    # Provider is in a named group
                    group_id = _escape_dot_label(prov_group)
                    lines.append(f"  {_service_provide_edge(group_id, svc_id)}")
                else:
                    # Provider is a root namespace node
                    node_id = _escape_dot_label(provider.fqn)
                    lines.append(f"  {_service_provide_edge(node_id, svc_id)}")

            for client in service.clients:
                client_group = extract_group_name(client.namespace)
                if client_group is not None:
                    # Client is in a named group
                    group_id = _escape_dot_label(client_group)
                    lines.append(f"  {_service_call_edge(group_id, svc_id)}")
                else:
                    # Client is a root namespace node
                    node_id = _escape_dot_label(client.fqn)
                    lines.append(f"  {_service_call_edge(node_id, svc_id)}")

        lines.append("")

    # Add inter-group actions
    if inter_actions:
        lines.append("  // Inter-group Actions")
        for action in inter_actions:
            act_id = _escape_dot_label(action.name)
            label = f"{action.name}\\n[{action.action_type}]"
            lines.append(f"  {_action_node_attrs(act_id, label)}")

            for server in action.servers:
                srv_group = extract_group_name(server.namespace)
                if srv_group is not None:
                    # Server is in a named group
                    group_id = _escape_dot_label(srv_group)
                    lines.append(f"  {_action_serve_edge(group_id, act_id)}")
                else:
                    # Server is a root namespace node
                    node_id = _escape_dot_label(server.fqn)
                    lines.append(f"  {_action_serve_edge(node_id, act_id)}")

            for client in action.clients:
                client_group = extract_group_name(client.namespace)
                if client_group is not None:
                    # Client is in a named group
                    group_id = _escape_dot_label(client_group)
                    lines.append(f"  {_action_call_edge(group_id, act_id)}")
                else:
                    # Client is a root namespace node
                    node_id = _escape_dot_label(client.fqn)
                    lines.append(f"  {_action_call_edge(node_id, act_id)}")

        lines.append("")

    lines.append("}")
    return "\n".join(lines)


def _generate_group_graph(graph: Graph, group_name: str | None, group_nodes: list[Node]) -> str:
    """
    Generate a DOT graph for a specific group with internal elements in a cluster.

    Args:
        graph: The complete graph
        group_name: Name of the group (or None for root namespace)
        group_nodes: List of nodes in this group

    Returns:
        DOT format string for the group graph
    """
    node_fqns = {node.fqn for node in group_nodes}
    display_name = group_name if group_name else "root"
    cluster_name = display_name.replace("/", "_")

    # Separate topics into internal vs external
    internal_topics = []
    external_topics = []
    for topic in graph.topics:
        pubs_in_group = [p for p in topic.publishers if p.fqn in node_fqns]
        subs_in_group = [s for s in topic.subscribers if s.fqn in node_fqns]

        if pubs_in_group or subs_in_group:
            # Check if all publishers and subscribers are in this group
            all_pubs_internal = all(p.fqn in node_fqns for p in topic.publishers)
            all_subs_internal = all(s.fqn in node_fqns for s in topic.subscribers)

            if all_pubs_internal and all_subs_internal:
                internal_topics.append(topic)
            else:
                external_topics.append(topic)

    # Separate services into internal vs external
    internal_services = []
    external_services = []
    for service in graph.services:
        provs_in_group = [p for p in service.providers if p.fqn in node_fqns]
        clients_in_group = [c for c in service.clients if c.fqn in node_fqns]

        if provs_in_group or clients_in_group:
            all_provs_internal = all(p.fqn in node_fqns for p in service.providers)
            all_clients_internal = all(c.fqn in node_fqns for c in service.clients)

            if all_provs_internal and all_clients_internal:
                internal_services.append(service)
            else:
                external_services.append(service)

    # Separate actions into internal vs external
    internal_actions = []
    external_actions = []
    for action in graph.actions:
        srvs_in_group = [s for s in action.servers if s.fqn in node_fqns]
        clients_in_group = [c for c in action.clients if c.fqn in node_fqns]

        if srvs_in_group or clients_in_group:
            all_srvs_internal = all(s.fqn in node_fqns for s in action.servers)
            all_clients_internal = all(c.fqn in node_fqns for c in action.clients)

            if all_srvs_internal and all_clients_internal:
                internal_actions.append(action)
            else:
                external_actions.append(action)

    # Build the graph
    lines = [f"digraph ROS2Group_{cluster_name} {{"]
    lines.append("  rankdir=LR;")
    lines.append("  node [shape=box];")
    lines.append(f'  label="Group: {display_name}";')
    lines.append("  labelloc=t;")
    lines.append("")

    # Start cluster for internal elements
    lines.append(f"  subgraph cluster_{cluster_name} {{")
    lines.append(f'    label="{display_name}";')
    lines.append("    style=filled;")
    lines.append("    fillcolor=lightgrey;")
    lines.append("")

    # Add group nodes inside cluster
    lines.append("    // Group Nodes")
    for node in group_nodes:
        node_id = _escape_dot_label(node.fqn)
        label = f"{node.fqn}\\n({node.package})"
        lines.append(f"    {_ros_node_attrs(node_id, label)}")

    # Add internal topics inside cluster
    if internal_topics:
        lines.append("")
        lines.append("    // Internal Topics")
        for topic in internal_topics:
            topic_id = _escape_dot_label(topic.name)
            label = f"{topic.name}\\n[{topic.msg_type}]"
            lines.append(f"    {_topic_node_attrs(topic_id, label)}")

    # Add internal services inside cluster
    if internal_services:
        lines.append("")
        lines.append("    // Internal Services")
        for service in internal_services:
            svc_id = _escape_dot_label(service.name)
            label = f"{service.name}\\n[{service.srv_type}]"
            lines.append(f"    {_service_node_attrs(svc_id, label)}")

    # Add internal actions inside cluster
    if internal_actions:
        lines.append("")
        lines.append("    // Internal Actions")
        for action in internal_actions:
            act_id = _escape_dot_label(action.name)
            label = f"{action.name}\\n[{action.action_type}]"
            lines.append(f"    {_action_node_attrs(act_id, label)}")

    lines.append("  }")
    lines.append("")

    # Add external topics outside cluster
    if external_topics:
        lines.append("  // External Topics")
        for topic in external_topics:
            topic_id = _escape_dot_label(topic.name)
            label = f"{topic.name}\\n[{topic.msg_type}]"
            lines.append(f"  {_topic_node_attrs(topic_id, label)}")
        lines.append("")

    # Add external services outside cluster
    if external_services:
        lines.append("  // External Services")
        for service in external_services:
            svc_id = _escape_dot_label(service.name)
            label = f"{service.name}\\n[{service.srv_type}]"
            lines.append(f"  {_service_node_attrs(svc_id, label)}")
        lines.append("")

    # Add external actions outside cluster
    if external_actions:
        lines.append("  // External Actions")
        for action in external_actions:
            act_id = _escape_dot_label(action.name)
            label = f"{action.name}\\n[{action.action_type}]"
            lines.append(f"  {_action_node_attrs(act_id, label)}")
        lines.append("")

    # Add edges for internal topics
    if internal_topics:
        lines.append("  // Internal Topic Edges")
        for topic in internal_topics:
            topic_id = _escape_dot_label(topic.name)
            for pub in topic.publishers:
                pub_id = _escape_dot_label(pub.fqn)
                lines.append(f"  {_topic_pub_edge(pub_id, topic_id)}")
            for sub in topic.subscribers:
                sub_id = _escape_dot_label(sub.fqn)
                lines.append(f"  {_topic_sub_edge(topic_id, sub_id)}")
        lines.append("")

    # Add edges for external topics (only for nodes in this group)
    if external_topics:
        lines.append("  // External Topic Edges")
        for topic in external_topics:
            topic_id = _escape_dot_label(topic.name)
            for pub in topic.publishers:
                if pub.fqn in node_fqns:
                    pub_id = _escape_dot_label(pub.fqn)
                    lines.append(f"  {_topic_pub_edge(pub_id, topic_id)}")
            for sub in topic.subscribers:
                if sub.fqn in node_fqns:
                    sub_id = _escape_dot_label(sub.fqn)
                    lines.append(f"  {_topic_sub_edge(topic_id, sub_id)}")
        lines.append("")

    # Add edges for internal services
    if internal_services:
        lines.append("  // Internal Service Edges")
        for service in internal_services:
            svc_id = _escape_dot_label(service.name)
            for prov in service.providers:
                prov_id = _escape_dot_label(prov.fqn)
                lines.append(f"  {_service_provide_edge(prov_id, svc_id)}")
            for client in service.clients:
                client_id = _escape_dot_label(client.fqn)
                lines.append(f"  {_service_call_edge(client_id, svc_id)}")
        lines.append("")

    # Add edges for external services (only for nodes in this group)
    if external_services:
        lines.append("  // External Service Edges")
        for service in external_services:
            svc_id = _escape_dot_label(service.name)
            for prov in service.providers:
                if prov.fqn in node_fqns:
                    prov_id = _escape_dot_label(prov.fqn)
                    lines.append(f"  {_service_provide_edge(prov_id, svc_id)}")
            for client in service.clients:
                if client.fqn in node_fqns:
                    client_id = _escape_dot_label(client.fqn)
                    lines.append(f"  {_service_call_edge(client_id, svc_id)}")
        lines.append("")

    # Add edges for internal actions
    if internal_actions:
        lines.append("  // Internal Action Edges")
        for action in internal_actions:
            act_id = _escape_dot_label(action.name)
            for srv in action.servers:
                srv_id = _escape_dot_label(srv.fqn)
                lines.append(f"  {_action_serve_edge(srv_id, act_id)}")
            for client in action.clients:
                client_id = _escape_dot_label(client.fqn)
                lines.append(f"  {_action_call_edge(client_id, act_id)}")
        lines.append("")

    # Add edges for external actions (only for nodes in this group)
    if external_actions:
        lines.append("  // External Action Edges")
        for action in external_actions:
            act_id = _escape_dot_label(action.name)
            for srv in action.servers:
                if srv.fqn in node_fqns:
                    srv_id = _escape_dot_label(srv.fqn)
                    lines.append(f"  {_action_serve_edge(srv_id, act_id)}")
            for client in action.clients:
                if client.fqn in node_fqns:
                    client_id = _escape_dot_label(client.fqn)
                    lines.append(f"  {_action_call_edge(client_id, act_id)}")
        lines.append("")

    lines.append("}")
    return "\n".join(lines)


def serialize_to_grouped_dot_files(graph: Graph, output_prefix: str) -> list[Path]:
    """
    Generate multiple DOT files showing grouped views of the ROS graph.

    Creates:
    - A top-level graph showing groups and inter-group connections
    - A graph per group showing internal and external connections

    Args:
        graph: The complete graph to serialize
        output_prefix: Path prefix for output files (e.g., "/path/to/output")
                      Will generate files like:
                      - {output_prefix}_toplevel.dot
                      - {output_prefix}_group_robot1.dot
                      - etc.

    Returns:
        List of Path objects for all generated files
    """
    # Group nodes by namespace
    groups = group_nodes_by_namespace(graph.nodes)

    generated_files = []

    # Generate top-level graph
    toplevel_dot = _generate_toplevel_graph(graph, groups)
    toplevel_path = Path(f"{output_prefix}_toplevel.dot")
    toplevel_path.write_text(toplevel_dot)
    generated_files.append(toplevel_path)

    # Generate per-group graphs (skip root namespace group)
    for group_name, group_nodes in groups.items():
        if group_name is not None:
            group_dot = _generate_group_graph(graph, group_name, group_nodes)
            # Sanitize group name for filename
            safe_group_name = group_name.replace("/", "_")
            group_path = Path(f"{output_prefix}_group_{safe_group_name}.dot")
            group_path.write_text(group_dot)
            generated_files.append(group_path)

    return generated_files
