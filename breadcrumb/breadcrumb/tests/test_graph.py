"""Tests for resolve_name_params(), expand_for_each_param(), filter_system_interfaces(), and extract_tf_roles() in breadcrumb.graph."""

import warnings

import pytest

from breadcrumb.graph import (
    Action,
    Graph,
    Node,
    Service,
    Topic,
    TopicConnection,
    expand_for_each_param,
    extract_tf_roles,
    filter_system_interfaces,
    resolve_name_params,
)
from breadcrumb.node_interface import ParameterDefinition


def _make_param(name: str, default_value=None, param_type: str = "string") -> ParameterDefinition:
    return ParameterDefinition(name=name, param_type=param_type, default_value=default_value)


class TestResolveNameParams:
    """Tests for resolve_name_params()."""

    def test_no_param_reference(self):
        """Names without ${param:...} are returned unchanged."""
        assert resolve_name_params("/cmd_vel", {}, {}) == "/cmd_vel"

    def test_single_partial_substitution(self):
        """A single ${param:...} embedded in a name is resolved."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {"id": "robot1"},
            {},
        )
        assert result == "/robot/robot1/cmd"

    def test_multiple_substitutions(self):
        """Multiple ${param:...} references in one name are all resolved."""
        result = resolve_name_params(
            "/${param:ns}/${param:id}/data",
            {"ns": "fleet", "id": "unit3"},
            {},
        )
        assert result == "/fleet/unit3/data"

    def test_full_substitution(self):
        """A name that is entirely a ${param:...} reference is resolved."""
        result = resolve_name_params(
            "${param:topic}",
            {"topic": "/my_topic"},
            {},
        )
        assert result == "/my_topic"

    def test_launch_param_priority_over_default(self):
        """Launch parameters take priority over interface defaults."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {"id": "launch_value"},
            {"id": _make_param("id", default_value="default_value")},
        )
        assert result == "/robot/launch_value/cmd"

    def test_falls_back_to_interface_default(self):
        """Falls back to interface default when no launch param is provided."""
        result = resolve_name_params(
            "/robot/${param:id}/cmd",
            {},
            {"id": _make_param("id", default_value="default_robot")},
        )
        assert result == "/robot/default_robot/cmd"

    def test_integer_param_auto_converted(self):
        """Integer default values are converted to strings."""
        result = resolve_name_params(
            "/robot/${param:port}/data",
            {},
            {"port": _make_param("port", default_value=42, param_type="integer")},
        )
        assert result == "/robot/42/data"

    def test_whitespace_tolerance(self):
        """Whitespace around the parameter name is tolerated."""
        result = resolve_name_params(
            "/robot/${param: robot_id }/cmd",
            {"robot_id": "r1"},
            {},
        )
        assert result == "/robot/r1/cmd"

    def test_unresolvable_left_as_is(self):
        """Unresolvable references are left as-is in the output."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/robot/${param:missing}/cmd",
                {},
                {},
            )
        assert result == "/robot/${param:missing}/cmd"

    def test_unresolvable_emits_warning(self):
        """Unresolvable references emit a UserWarning."""
        with pytest.warns(UserWarning, match="missing"):
            resolve_name_params(
                "/robot/${param:missing}/cmd",
                {},
                {},
            )

    def test_mixed_resolved_and_unresolved(self):
        """One param resolves, another doesn't — partial resolution."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/${param:ns}/${param:missing}/data",
                {"ns": "found"},
                {},
            )
        assert result == "/found/${param:missing}/data"

    def test_non_param_syntax_unchanged(self):
        """Non-param ${...} syntax (e.g., ${env:HOME}) is left untouched."""
        result = resolve_name_params(
            "/robot/${env:HOME}/cmd",
            {},
            {},
        )
        assert result == "/robot/${env:HOME}/cmd"

    def test_none_default_falls_through(self):
        """A parameter with default_value=None is treated as unresolvable."""
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            result = resolve_name_params(
                "/robot/${param:id}/cmd",
                {},
                {"id": _make_param("id", default_value=None)},
            )
        assert result == "/robot/${param:id}/cmd"


class TestExpandForEachParam:
    """Tests for expand_for_each_param()."""

    def test_no_for_each_param(self):
        """Names without ${for_each_param:...} return a single-element list."""
        result = expand_for_each_param("/cmd_vel", {}, {})
        assert result == ["/cmd_vel"]

    def test_no_for_each_param_with_param_ref(self):
        """Names with only ${param:...} are resolved and returned as single-element list."""
        result = expand_for_each_param(
            "/robot/${param:id}/cmd",
            {"id": "robot1"},
            {},
        )
        assert result == ["/robot/robot1/cmd"]

    def test_basic_expansion_from_launch_params(self):
        """Basic expansion from a launch parameter list."""
        result = expand_for_each_param(
            "/cameras/${for_each_param:camera_names}/image",
            {"camera_names": ["front", "rear", "left"]},
            {},
        )
        assert result == [
            "/cameras/front/image",
            "/cameras/rear/image",
            "/cameras/left/image",
        ]

    def test_expansion_from_interface_defaults(self):
        """Expansion using interface parameter default values."""
        result = expand_for_each_param(
            "/sensors/${for_each_param:sensor_list}/data",
            {},
            {"sensor_list": _make_param("sensor_list", default_value=["lidar", "radar"], param_type="string_array")},
        )
        assert result == [
            "/sensors/lidar/data",
            "/sensors/radar/data",
        ]

    def test_launch_params_priority_over_defaults(self):
        """Launch parameters take priority over interface defaults."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": ["alpha", "beta"]},
            {"names": _make_param("names", default_value=["x", "y", "z"], param_type="string_array")},
        )
        assert result == [
            "/topic/alpha/out",
            "/topic/beta/out",
        ]

    def test_coexistence_with_param_ref(self):
        """${for_each_param:...} and ${param:...} coexist in the same name."""
        result = expand_for_each_param(
            "/${param:ns}/${for_each_param:items}/data",
            {"ns": "fleet", "items": ["cam1", "cam2"]},
            {},
        )
        assert result == [
            "/fleet/cam1/data",
            "/fleet/cam2/data",
        ]

    def test_single_element_array(self):
        """A single-element array produces one result."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": ["only"]},
            {},
        )
        assert result == ["/topic/only/out"]

    def test_empty_array(self):
        """An empty array produces an empty list."""
        result = expand_for_each_param(
            "/topic/${for_each_param:names}/out",
            {"names": []},
            {},
        )
        assert result == []

    def test_non_list_param_warns_and_falls_back(self):
        """A non-list parameter emits a warning and returns single-element fallback."""
        with pytest.warns(UserWarning, match="not a list"):
            result = expand_for_each_param(
                "/topic/${for_each_param:names}/out",
                {"names": "not_a_list"},
                {},
            )
        assert len(result) == 1

    def test_missing_param_warns_and_falls_back(self):
        """A missing parameter emits a warning and returns single-element fallback."""
        with pytest.warns(UserWarning, match="Could not resolve for_each_param"):
            result = expand_for_each_param(
                "/topic/${for_each_param:missing}/out",
                {},
                {},
            )
        assert len(result) == 1

    def test_whitespace_tolerance(self):
        """Whitespace around the parameter name in the token is tolerated."""
        result = expand_for_each_param(
            "/topic/${for_each_param: names }/out",
            {"names": ["a", "b"]},
            {},
        )
        assert result == ["/topic/a/out", "/topic/b/out"]


def _make_node(name: str) -> Node:
    return Node(fqn=f"/{name}", name=name, namespace=None, package="test_pkg")


class TestFilterSystemInterfaces:
    """Tests for filter_system_interfaces()."""

    def test_always_hides_rosout(self):
        """The /rosout topic is always removed, even with subscribers."""
        node_a = _make_node("a")
        node_b = _make_node("b")
        graph = Graph(
            nodes=[node_a, node_b],
            topics=[
                Topic(
                    name="/rosout",
                    msg_type="rcl_interfaces/msg/Log",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_b)],
                ),
                Topic(name="/cmd_vel", msg_type="geometry_msgs/msg/Twist"),
            ],
        )
        filter_system_interfaces(graph)
        assert [t.name for t in graph.topics] == ["/cmd_vel"]

    def test_always_hides_parameter_events(self):
        """The /parameter_events topic is always removed."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/parameter_events",
                    msg_type="rcl_interfaces/msg/ParameterEvent",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert graph.topics == []

    def test_hides_unconnected_lifecycle_topic(self):
        """Lifecycle topics with only publishers (no subscribers) are removed."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/a/transition_event",
                    msg_type="lifecycle_msgs/msg/TransitionEvent",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert graph.topics == []

    def test_keeps_connected_lifecycle_topic(self):
        """Lifecycle topics with both publishers and subscribers are kept."""
        node_a = _make_node("a")
        node_monitor = _make_node("monitor")
        graph = Graph(
            nodes=[node_a, node_monitor],
            topics=[
                Topic(
                    name="/a/transition_event",
                    msg_type="lifecycle_msgs/msg/TransitionEvent",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_monitor)],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert len(graph.topics) == 1
        assert graph.topics[0].name == "/a/transition_event"

    def test_hides_unconnected_lifecycle_service(self):
        """Lifecycle services with only a provider (no clients) are removed."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            services=[
                Service(name="/a/change_state", srv_type="lifecycle_msgs/srv/ChangeState", providers=[node_a]),
                Service(name="/a/get_state", srv_type="lifecycle_msgs/srv/GetState", providers=[node_a]),
            ],
        )
        filter_system_interfaces(graph)
        assert graph.services == []

    def test_keeps_connected_lifecycle_service(self):
        """Lifecycle services with both provider and client are kept."""
        node_a = _make_node("a")
        node_manager = _make_node("lifecycle_manager")
        graph = Graph(
            nodes=[node_a, node_manager],
            services=[
                Service(
                    name="/a/change_state",
                    srv_type="lifecycle_msgs/srv/ChangeState",
                    providers=[node_a],
                    clients=[node_manager],
                ),
                Service(name="/a/get_state", srv_type="lifecycle_msgs/srv/GetState", providers=[node_a]),
            ],
        )
        filter_system_interfaces(graph)
        assert len(graph.services) == 1
        assert graph.services[0].name == "/a/change_state"

    def test_hides_unconnected_parameter_service(self):
        """Parameter services with only a provider are removed."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            services=[
                Service(
                    name="/a/describe_parameters",
                    srv_type="rcl_interfaces/srv/DescribeParameters",
                    providers=[node_a],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert graph.services == []

    def test_preserves_user_topics(self):
        """Non-system topics are never filtered."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert len(graph.topics) == 1

    def test_preserves_user_services(self):
        """Non-system services are never filtered."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            services=[
                Service(name="/a/reset", srv_type="std_srvs/srv/Trigger", providers=[node_a]),
            ],
        )
        filter_system_interfaces(graph)
        assert len(graph.services) == 1

    def test_mixed_graph(self):
        """Filtering works correctly on a graph with both system and user interfaces."""
        node_a = _make_node("a")
        node_b = _make_node("b")
        node_manager = _make_node("lifecycle_manager")
        graph = Graph(
            nodes=[node_a, node_b, node_manager],
            topics=[
                # User topic — kept
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_b)],
                ),
                # Always hidden
                Topic(
                    name="/rosout",
                    msg_type="rcl_interfaces/msg/Log",
                    publishers=[TopicConnection(node=node_a), TopicConnection(node=node_b)],
                ),
                # Unconnected system topic — hidden
                Topic(
                    name="/a/transition_event",
                    msg_type="lifecycle_msgs/msg/TransitionEvent",
                    publishers=[TopicConnection(node=node_a)],
                ),
                # Connected system topic — kept
                Topic(
                    name="/b/transition_event",
                    msg_type="lifecycle_msgs/msg/TransitionEvent",
                    publishers=[TopicConnection(node=node_b)],
                    subscribers=[TopicConnection(node=node_manager)],
                ),
            ],
            services=[
                # User service — kept
                Service(name="/a/reset", srv_type="std_srvs/srv/Trigger", providers=[node_a]),
                # Connected system service — kept
                Service(
                    name="/a/change_state",
                    srv_type="lifecycle_msgs/srv/ChangeState",
                    providers=[node_a],
                    clients=[node_manager],
                ),
                # Unconnected system service — hidden
                Service(
                    name="/a/get_state",
                    srv_type="lifecycle_msgs/srv/GetState",
                    providers=[node_a],
                ),
            ],
        )
        filter_system_interfaces(graph)
        assert [t.name for t in graph.topics] == ["/cmd_vel", "/b/transition_event"]
        assert [s.name for s in graph.services] == ["/a/reset", "/a/change_state"]


class TestExtractTfRoles:
    """Tests for extract_tf_roles()."""

    def test_broadcaster_sets_flag(self):
        """A node publishing /tf gets tf_broadcaster=True."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/tf",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_broadcaster is True
        assert node_a.tf_listener is False
        assert node_a.tf_static_broadcaster is False
        assert graph.topics == []

    def test_static_broadcaster_sets_flag(self):
        """A node publishing /tf_static gets tf_static_broadcaster=True."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/tf_static",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_static_broadcaster is True
        assert node_a.tf_broadcaster is False
        assert node_a.tf_listener is False
        assert graph.topics == []

    def test_listener_sets_flag(self):
        """A node subscribing /tf gets tf_listener=True."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/tf",
                    msg_type="tf2_msgs/msg/TFMessage",
                    subscribers=[TopicConnection(node=node_a)],
                ),
                Topic(
                    name="/tf_static",
                    msg_type="tf2_msgs/msg/TFMessage",
                    subscribers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_listener is True
        assert node_a.tf_broadcaster is False
        assert node_a.tf_static_broadcaster is False
        assert graph.topics == []

    def test_all_roles(self):
        """A node with listener + broadcaster + static broadcaster gets all three flags."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/tf",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_a)],
                ),
                Topic(
                    name="/tf_static",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_listener is True
        assert node_a.tf_broadcaster is True
        assert node_a.tf_static_broadcaster is True
        assert graph.topics == []

    def test_multiple_nodes_different_roles(self):
        """Different nodes get their respective TF roles."""
        node_broadcaster = _make_node("broadcaster")
        node_listener = _make_node("listener")
        node_plain = _make_node("plain")
        graph = Graph(
            nodes=[node_broadcaster, node_listener, node_plain],
            topics=[
                Topic(
                    name="/tf",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_broadcaster)],
                    subscribers=[TopicConnection(node=node_listener)],
                ),
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_plain)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_broadcaster.tf_broadcaster is True
        assert node_broadcaster.tf_listener is False
        assert node_listener.tf_listener is True
        assert node_listener.tf_broadcaster is False
        assert node_plain.tf_listener is False
        assert node_plain.tf_broadcaster is False
        # /cmd_vel preserved, /tf removed
        assert [t.name for t in graph.topics] == ["/cmd_vel"]

    def test_preserves_non_tf_topics(self):
        """Non-TF topics are not removed."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_a)],
                ),
                Topic(
                    name="/tf",
                    msg_type="tf2_msgs/msg/TFMessage",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert [t.name for t in graph.topics] == ["/cmd_vel"]

    def test_ignores_non_tf_msg_type_on_tf_topic(self):
        """A topic named /tf but with a non-TF message type is not treated as TF."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/tf",
                    msg_type="std_msgs/msg/String",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_broadcaster is False
        assert [t.name for t in graph.topics] == ["/tf"]

    def test_no_tf_topics_is_noop(self):
        """A graph with no TF topics is unchanged."""
        node_a = _make_node("a")
        graph = Graph(
            nodes=[node_a],
            topics=[
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_a)],
                ),
            ],
        )
        extract_tf_roles(graph)
        assert node_a.tf_listener is False
        assert node_a.tf_broadcaster is False
        assert node_a.tf_static_broadcaster is False
        assert len(graph.topics) == 1
