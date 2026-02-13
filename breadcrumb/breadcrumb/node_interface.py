from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
import yaml

from typing import Any, List


class ReliabilityPolicy(Enum):
    RELIABLE = "RELIABLE"
    BEST_EFFORT = "BEST_EFFORT"


class DurabilityPolicy(Enum):
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"
    VOLATILE = "VOLATILE"


class LivelinessPolicy(Enum):
    AUTOMATIC = "AUTOMATIC"
    MANUAL_BY_TOPIC = "MANUAL_BY_TOPIC"


@dataclass
class QoS:
    """QoS settings for a publisher or subscriber.

    Fields may contain:
    - Resolved values (enums, ints, "ALL")
    - Unresolved parameter references (${param:name} strings)
    """

    history: int | str  # integer >= 1 OR "ALL" OR ${param:name}
    reliability: ReliabilityPolicy | str  # Enum OR ${param:name}
    durability: DurabilityPolicy | str | None = None
    deadline_ms: int | str | None = None
    lifespan_ms: int | str | None = None
    liveliness: LivelinessPolicy | str | None = None
    lease_duration_ms: int | str | None = None


@dataclass
class ParameterDefinition:
    """Definition of a node parameter with its default value."""

    name: str
    param_type: str
    default_value: Any
    description: str | None = None
    read_only: bool = False


@dataclass
class NodeInfo:
    name: str
    package: str
    plugin: str | None = None


@dataclass
class Publisher:
    topic: str
    type: str
    qos: QoS | None = None


@dataclass
class Subscriber:
    topic: str
    type: str
    qos: QoS | None = None


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
    info: NodeInfo
    publishers: List[Publisher] = field(default_factory=list)
    subscribers: List[Subscriber] = field(default_factory=list)
    services: List[Service] = field(default_factory=list)
    service_clients: List[ServiceClient] = field(default_factory=list)
    actions: List[Action] = field(default_factory=list)
    action_clients: List[ActionClient] = field(default_factory=list)
    parameters: dict[str, ParameterDefinition] = field(default_factory=dict)


_PARAM_SUB_PATTERN = re.compile(r"\$\{param:\s*([^}]+?)\s*\}")


def _is_param_reference(value: Any) -> bool:
    """Check if a value is a parameter reference like ${param:name}."""
    if isinstance(value, str):
        return bool(re.match(r"^\$\{param:[^}]+\}$", value))
    return False


def _contains_param_reference(value: Any) -> bool:
    """Check if a value contains any ${param:name} reference (partial or full)."""
    if isinstance(value, str):
        return bool(_PARAM_SUB_PATTERN.search(value))
    return False


def _extract_param_name(value: str) -> str | None:
    """Extract parameter name from ${param:name} reference."""
    match = re.match(r"^\$\{param:([^}]+)\}$", value)
    return match.group(1) if match else None


def parse_qos_raw(qos_data: dict[str, Any] | None) -> QoS | None:
    """
    Parse QoS from YAML data, preserving ${param:name} references.

    Args:
        qos_data: Dictionary containing QoS fields from YAML

    Returns:
        QoS object with values or parameter references, or None if no valid QoS
    """
    if qos_data is None:
        return None

    # Skip old-style profile-only QoS (like SystemDefaultsQoS)
    if "profile" in qos_data and "history" not in qos_data:
        return None

    # Required fields
    history_raw = qos_data.get("history")
    reliability_raw = qos_data.get("reliability")

    if history_raw is None or reliability_raw is None:
        return None

    # Parse history: integer >= 1 OR "ALL" OR ${param:name}
    if _is_param_reference(history_raw):
        history: int | str = history_raw
    elif history_raw == "ALL":
        history = "ALL"
    elif isinstance(history_raw, int) and history_raw >= 1:
        history = history_raw
    else:
        return None

    # Parse reliability: RELIABLE or BEST_EFFORT OR ${param:name}
    if _is_param_reference(reliability_raw):
        reliability: ReliabilityPolicy | str = reliability_raw
    elif reliability_raw in ("RELIABLE", "BEST_EFFORT"):
        reliability = ReliabilityPolicy(reliability_raw)
    else:
        return None

    # Parse optional durability
    durability: DurabilityPolicy | str | None = None
    durability_raw = qos_data.get("durability")
    if durability_raw is not None:
        if _is_param_reference(durability_raw):
            durability = durability_raw
        elif durability_raw in ("TRANSIENT_LOCAL", "VOLATILE"):
            durability = DurabilityPolicy(durability_raw)

    # Parse optional deadline_ms
    deadline_ms: int | str | None = None
    deadline_raw = qos_data.get("deadline_ms")
    if deadline_raw is not None:
        if _is_param_reference(deadline_raw):
            deadline_ms = deadline_raw
        elif isinstance(deadline_raw, int) and deadline_raw >= 0:
            deadline_ms = deadline_raw

    # Parse optional lifespan_ms
    lifespan_ms: int | str | None = None
    lifespan_raw = qos_data.get("lifespan_ms")
    if lifespan_raw is not None:
        if _is_param_reference(lifespan_raw):
            lifespan_ms = lifespan_raw
        elif isinstance(lifespan_raw, int) and lifespan_raw >= 0:
            lifespan_ms = lifespan_raw

    # Parse optional liveliness
    liveliness: LivelinessPolicy | str | None = None
    liveliness_raw = qos_data.get("liveliness")
    if liveliness_raw is not None:
        if _is_param_reference(liveliness_raw):
            liveliness = liveliness_raw
        elif liveliness_raw in ("AUTOMATIC", "MANUAL_BY_TOPIC"):
            liveliness = LivelinessPolicy(liveliness_raw)

    # Parse optional lease_duration_ms
    lease_duration_ms: int | str | None = None
    lease_raw = qos_data.get("lease_duration_ms")
    if lease_raw is not None:
        if _is_param_reference(lease_raw):
            lease_duration_ms = lease_raw
        elif isinstance(lease_raw, int) and lease_raw >= 0:
            lease_duration_ms = lease_raw

    return QoS(
        history=history,
        reliability=reliability,
        durability=durability,
        deadline_ms=deadline_ms,
        lifespan_ms=lifespan_ms,
        liveliness=liveliness,
        lease_duration_ms=lease_duration_ms,
    )


def parse_parameters(params_data: dict[str, Any] | None) -> dict[str, ParameterDefinition]:
    """
    Parse parameter definitions from YAML data.

    Args:
        params_data: Dictionary containing parameter definitions from YAML

    Returns:
        Dictionary mapping parameter names to ParameterDefinition objects
    """
    if params_data is None:
        return {}

    result = {}
    for name, param_info in params_data.items():
        if isinstance(param_info, dict):
            result[name] = ParameterDefinition(
                name=name,
                param_type=param_info.get("type", "string"),
                default_value=param_info.get("default_value"),
                description=param_info.get("description"),
                read_only=param_info.get("read_only", False),
            )
    return result


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
    info = NodeInfo(name=node_data["name"], package=node_data["package"], plugin=node_data.get("plugin"))

    # Parse parameters
    parameters = parse_parameters(data.get("parameters"))

    # Parse publishers with QoS
    publishers = [
        Publisher(
            topic=pub["topic"],
            type=pub["type"],
            qos=parse_qos_raw(pub.get("qos")),
        )
        for pub in (data.get("publishers") or [])
    ]

    # Parse subscribers with QoS
    subscribers = [
        Subscriber(
            topic=sub["topic"],
            type=sub["type"],
            qos=parse_qos_raw(sub.get("qos")),
        )
        for sub in (data.get("subscribers") or [])
    ]

    # Parse services
    services = [Service(name=svc["name"], type=svc["type"]) for svc in (data.get("services") or [])]

    # Parse service clients
    service_clients = [ServiceClient(name=svc["name"], type=svc["type"]) for svc in (data.get("service_clients") or [])]

    # Parse actions
    actions = [Action(name=act["name"], type=act["type"]) for act in (data.get("actions") or [])]

    # Parse action clients
    action_clients = [ActionClient(name=act["name"], type=act["type"]) for act in (data.get("action_clients") or [])]

    return NodeInterface(
        info=info,
        publishers=publishers,
        subscribers=subscribers,
        services=services,
        service_clients=service_clients,
        actions=actions,
        action_clients=action_clients,
        parameters=parameters,
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

    # If still not found, try breadcrumb's built-in interfaces as a fallback
    if yaml_path is None:
        yaml_path = find_interface_in_package("breadcrumb", executable=executable, plugin=plugin)

    # If still not found, raise an error
    if yaml_path is None:
        search_criteria = f"executable='{executable}'" if executable else f"plugin='{plugin}'"
        packages_searched = [package]
        if launching_package is not None:
            packages_searched.append(launching_package)
        packages_searched.append("breadcrumb")
        raise FileNotFoundError(
            f"Could not find node interface YAML for {search_criteria} in packages: {packages_searched}"
        )

    return parse_node_interface(yaml_path)
