"""Serialization functions for converting graph objects to various formats."""

from pathlib import Path

from .graph import Action, Graph, Node, Service, Topic

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
        label = f"{node.name}\\n({node.package})"
        lines.append(f'  "{node_id}" [label="{_escape_dot_label(label)}", style=filled, fillcolor=lightblue];')

    lines.append("")

    # Add topics as separate nodes
    lines.append("  // Topics")
    for topic in graph.topics:
        topic_id = _escape_dot_label(topic.name)
        label = f"{topic.name}\\n[{topic.msg_type}]"
        lines.append(
            f'  "{topic_id}" [label="{_escape_dot_label(label)}", shape=ellipse, style=filled, fillcolor=lightgreen];'
        )

        # Connect publishers to topic
        for pub in topic.publishers:
            pub_id = _escape_dot_label(pub.fqn)
            lines.append(f'  "{pub_id}" -> "{topic_id}" [label="pub"];')

        # Connect topic to subscribers
        for sub in topic.subscribers:
            sub_id = _escape_dot_label(sub.fqn)
            lines.append(f'  "{topic_id}" -> "{sub_id}" [label="sub"];')

    lines.append("")

    # Add services
    lines.append("  // Services")
    for service in graph.services:
        svc_id = _escape_dot_label(service.name)
        label = f"{service.name}\\n[{service.srv_type}]"
        lines.append(
            f'  "{svc_id}" [label="{_escape_dot_label(label)}", shape=diamond, style=filled, fillcolor=lightyellow];'
        )

        # Connect providers to service
        for provider in service.providers:
            prov_id = _escape_dot_label(provider.fqn)
            lines.append(f'  "{prov_id}" -> "{svc_id}" [label="provide", style=dashed];')

        # Connect service to clients
        for client in service.clients:
            client_id = _escape_dot_label(client.fqn)
            lines.append(f'  "{svc_id}" -> "{client_id}" [label="call", style=dashed, dir=back];')

    lines.append("")

    # Add actions
    lines.append("  // Actions")
    for action in graph.actions:
        act_id = _escape_dot_label(action.name)
        label = f"{action.name}\\n[{action.action_type}]"
        lines.append(
            f'  "{act_id}" [label="{_escape_dot_label(label)}", shape=hexagon, style=filled, fillcolor=lightcoral];'
        )

        # Connect servers to action
        for server in action.servers:
            srv_id = _escape_dot_label(server.fqn)
            lines.append(f'  "{srv_id}" -> "{act_id}" [label="serve", style=dotted];')

        # Connect action to clients (with reversed arrow direction)
        for client in action.clients:
            client_id = _escape_dot_label(client.fqn)
            lines.append(f'  "{act_id}" -> "{client_id}" [label="call", style=dotted, dir=back];')

    lines.append("}")
    return "\n".join(lines)


def _escape_mermaid_label(text: str) -> str:
    """Escape special characters for Mermaid labels."""
    return (
        text.replace('"', "#quot;")
        .replace("[", "#91;")
        .replace("]", "#93;")
        .replace("(", "#40;")
        .replace(")", "#41;")
        .replace("{", "#123;")
        .replace("}", "#125;")
        .replace("<", "#60;")
        .replace(">", "#62;")
    )


def _mermaid_id(text: str) -> str:
    """Convert a name to a valid Mermaid node ID."""
    # Replace all special characters with underscores and ensure valid ID
    result = text.replace("/", "_").replace("~", "_").replace(".", "_").replace("-", "_")
    result = result.replace("(", "_").replace(")", "_").replace("[", "_").replace("]", "_")
    result = result.replace("{", "_").replace("}", "_").replace("<", "_").replace(">", "_")
    result = result.replace(" ", "_").lstrip("_")
    # Ensure we have a valid ID (not empty)
    return result if result else "node"
