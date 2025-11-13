from dataclasses import dataclass, field
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from typing import List


@dataclass
class NodeInfo:
    name: str
    package: str
    plugin: str | None = None


@dataclass
class Publisher:
    topic: str
    type: str


@dataclass
class Subscriber:
    topic: str
    type: str


@dataclass
class Service:
    name: str
    type: str


@dataclass
class ServiceClient:
    name: str
    type: str


@dataclass
class Action:
    name: str
    type: str


@dataclass
class ActionClient:
    name: str
    type: str


@dataclass
class NodeInterface:
    node: NodeInfo
    publishers: List[Publisher] = field(default_factory=list)
    subscribers: List[Subscriber] = field(default_factory=list)
    services: List[Service] = field(default_factory=list)
    service_clients: List[ServiceClient] = field(default_factory=list)
    actions: List[Action] = field(default_factory=list)
    action_clients: List[ActionClient] = field(default_factory=list)


def parse_node_interface(yaml_path: Path) -> NodeInterface:
    """
    Parse a ROS2 node interface YAML file into dataclasses.

    Args:
        yaml_path: Path to the YAML file to parse

    Returns:
        NodeInterface object containing all parsed data
    """

    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    # Parse node info
    node_data = data.get("node", {})
    node = NodeInfo(name=node_data["name"], package=node_data["package"], plugin=node_data.get("plugin"))

    # Parse publishers
    publishers = [Publisher(topic=pub["topic"], type=pub["type"]) for pub in (data.get("publishers") or [])]

    # Parse subscribers
    subscribers = [Subscriber(topic=sub["topic"], type=sub["type"]) for sub in (data.get("subscribers") or [])]

    # Parse services
    services = [Service(name=svc["name"], type=svc["type"]) for svc in (data.get("services") or [])]

    # Parse service clients
    service_clients = [ServiceClient(name=svc["name"], type=svc["type"]) for svc in (data.get("service_clients") or [])]

    # Parse actions
    actions = [Action(name=act["name"], type=act["type"]) for act in (data.get("actions") or [])]

    # Parse action clients
    action_clients = [ActionClient(name=act["name"], type=act["type"]) for act in (data.get("action_clients") or [])]

    return NodeInterface(
        node=node,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
    )


def get_node_interface_name(yaml_path: Path) -> str | None:
    """
    Read a node interface YAML file and return the node name.

    Args:
        yaml_path: Path to the YAML file

    Returns:
        The node name, or None if not found
    """
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
            node_data = data.get("node", {})
            return node_data.get("name")
    except (yaml.YAMLError, OSError):
        return None


def get_node_interface_plugin(yaml_path: Path) -> str | None:
    """
    Read a node interface YAML file and return the node plugin.

    Args:
        yaml_path: Path to the YAML file

    Returns:
        The node plugin, or None if not found
    """
    try:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
            node_data = data.get("node", {})
            return node_data.get("plugin")
    except (yaml.YAMLError, OSError, KeyError):
        return None


def find_interface_in_package(
    package: str,
    *,
    executable: str | None = None,
    plugin: str | None = None,
) -> Path | None:
    """
    Search for a node interface YAML file in a package's interfaces directory.

    Args:
        package: The ROS2 package name to search in
        executable: Optional executable name to match (looks for {executable}.yaml)
        plugin: Optional plugin name to match (searches YAML files for matching plugin field)

    Returns:
        Path to the matching YAML file, or None if not found
    """
    try:
        pkg_share = Path(get_package_share_directory(package))
        interfaces_dir = pkg_share / "interfaces"

        if not interfaces_dir.exists():
            return None

        if executable is not None:
            for yaml_file in interfaces_dir.glob("*.yaml"):
                file_name = get_node_interface_name(yaml_file)
                if file_name == executable:
                    return yaml_file

        if plugin is not None:
            for yaml_file in interfaces_dir.glob("*.yaml"):
                file_plugin = get_node_interface_plugin(yaml_file)
                if file_plugin == plugin:
                    return yaml_file

        return None
    except Exception:
        # Package not found or other error
        return None


def load_node_interface(
    package: str,
    *,
    executable: str | None = None,
    plugin: str | None = None,
    launching_package: str | None = None,
) -> NodeInterface:
    if executable is None and plugin is None:
        raise ValueError("One of executable or plugin must be provided!")

    # First, try to find the interface in the main package
    yaml_path = find_interface_in_package(package, executable=executable, plugin=plugin)

    # If not found and launching_package is provided, try there
    if yaml_path is None and launching_package is not None:
        yaml_path = find_interface_in_package(launching_package, executable=executable, plugin=plugin)

    # If still not found, raise an error
    if yaml_path is None:
        search_criteria = f"executable='{executable}'" if executable else f"plugin='{plugin}'"
        packages_searched = [package]
        if launching_package is not None:
            packages_searched.append(launching_package)
        raise FileNotFoundError(
            f"Could not find node interface YAML for {search_criteria} in packages: {packages_searched}"
        )

    return parse_node_interface(yaml_path)
