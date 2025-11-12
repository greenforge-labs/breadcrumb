"""Data structures for representing ROS 2 graphs."""

from dataclasses import dataclass, field

from typing import Any, Optional


@dataclass
class Node:
    """
    Represents a ROS 2 node in the graph.

    Attributes:
        id: Fully qualified name (e.g., "/cleaning/my_node" or "my_node")
        package: ROS package name
        executable: Executable name (for regular nodes) or plugin name (for composable)
        name: Node name (without namespace)
        namespace: Node namespace (e.g., "cleaning" or "robot1/sensors")
        is_composable: True if this is a composable node
        parameters: Runtime parameters from launch file
        remappings: Topic/service/action remappings from launch file
    """

    id: str
    package: str
    executable: str
    name: str
    namespace: Optional[str] = None
    is_composable: bool = False
    parameters: Optional[dict[str, Any]] = None
    remappings: Optional[dict[str, str]] = None


@dataclass
class Topic:
    """
    Represents a ROS 2 topic in the graph.

    Attributes:
        name: Fully resolved topic name (e.g., "/cleaning/cmd_vel")
        msg_type: Message type (e.g., "geometry_msgs/msg/Twist")
        qos: QoS profile information
        publishers: List of node IDs that publish to this topic
        subscribers: List of node IDs that subscribe to this topic
    """

    name: str
    msg_type: str
    qos: Optional[dict[str, Any]] = None
    publishers: list[str] = field(default_factory=list)
    subscribers: list[str] = field(default_factory=list)


@dataclass
class Service:
    """
    Represents a ROS 2 service in the graph.

    Attributes:
        name: Fully resolved service name (e.g., "/cleaning/reset")
        srv_type: Service type (e.g., "std_srvs/srv/Empty")
        qos: QoS profile information
        servers: List of node IDs that provide this service
        clients: List of node IDs that call this service
    """

    name: str
    srv_type: str
    qos: Optional[dict[str, Any]] = None
    servers: list[str] = field(default_factory=list)
    clients: list[str] = field(default_factory=list)


@dataclass
class Action:
    """
    Represents a ROS 2 action in the graph.

    Attributes:
        name: Fully resolved action name (e.g., "/navigate_to_pose")
        action_type: Action type (e.g., "nav2_msgs/action/NavigateToPose")
        servers: List of node IDs that provide this action
        clients: List of node IDs that call this action
    """

    name: str
    action_type: str
    servers: list[str] = field(default_factory=list)
    clients: list[str] = field(default_factory=list)


@dataclass
class RosGraph:
    """
    Complete ROS 2 computation graph.

    Attributes:
        nodes: All nodes in the graph
        topics: All topics with their publishers and subscribers
        services: All services with their servers and clients
        actions: All actions with their servers and clients
    """

    nodes: list[Node] = field(default_factory=list)
    topics: list[Topic] = field(default_factory=list)
    services: list[Service] = field(default_factory=list)
    actions: list[Action] = field(default_factory=list)

    def get_node_by_id(self, node_id: str) -> Optional[Node]:
        """Find a node by its fully qualified ID."""
        for node in self.nodes:
            if node.id == node_id:
                return node
        return None

    def get_topic_by_name(self, topic_name: str) -> Optional[Topic]:
        """Find a topic by its fully resolved name."""
        for topic in self.topics:
            if topic.name == topic_name:
                return topic
        return None

    def get_service_by_name(self, service_name: str) -> Optional[Service]:
        """Find a service by its fully resolved name."""
        for service in self.services:
            if service.name == service_name:
                return service
        return None

    def get_action_by_name(self, action_name: str) -> Optional[Action]:
        """Find an action by its fully resolved name."""
        for action in self.actions:
            if action.name == action_name:
                return action
        return None
