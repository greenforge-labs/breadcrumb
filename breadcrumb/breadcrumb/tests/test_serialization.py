"""Tests for TF bubble rendering in breadcrumb.serialization."""

from breadcrumb.graph import Graph, Node, Topic, TopicConnection
from breadcrumb.serialization import serialize_graph, serialize_to_dot


def _make_node(name: str, **kwargs) -> Node:
    return Node(fqn=f"/{name}", name=name, namespace=None, package="test_pkg", **kwargs)


class TestTfBubbleDotRendering:
    """Tests for TF role bubbles in DOT output."""

    def test_broadcaster_bubble_in_dot(self):
        """A node with tf_broadcaster=True renders a TF Broadcaster bubble."""
        node = _make_node("broadcaster", tf_broadcaster=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "TF Broadcaster" in dot
        assert "lightsalmon" in dot
        assert "TF Listener" not in dot
        assert "TF Static Pub" not in dot

    def test_listener_bubble_in_dot(self):
        """A node with tf_listener=True renders a TF Listener bubble."""
        node = _make_node("listener", tf_listener=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "TF Listener" in dot
        assert "plum" in dot
        assert "TF Broadcaster" not in dot

    def test_static_broadcaster_bubble_in_dot(self):
        """A node with tf_static_broadcaster=True renders a TF Static Pub bubble."""
        node = _make_node("static_pub", tf_static_broadcaster=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "TF Static Pub" in dot
        assert "peachpuff" in dot

    def test_all_bubbles_in_dot(self):
        """A node with all TF roles renders all three bubbles."""
        node = _make_node("all_tf", tf_listener=True, tf_broadcaster=True, tf_static_broadcaster=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "TF Listener" in dot
        assert "TF Broadcaster" in dot
        assert "TF Static Pub" in dot

    def test_no_tf_roles_no_bubbles(self):
        """A node without TF roles has no TF bubbles."""
        node = _make_node("plain")
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "TF Listener" not in dot
        assert "TF Broadcaster" not in dot
        assert "TF Static Pub" not in dot

    def test_bubbles_inside_node_cluster(self):
        """TF bubbles are rendered inside the node's subgraph cluster."""
        node = _make_node("my_node", tf_listener=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)

        # The bubble ID should reference the node
        assert "/my_node__tf_listener" in dot

        # The edge connects node to its bubble
        assert '"/my_node" -> "/my_node__tf_listener"' in dot

    def test_bubble_edge_is_thin_and_arrowless(self):
        """TF bubble edges have no arrowhead and thin grey styling."""
        node = _make_node("a", tf_broadcaster=True)
        graph = Graph(nodes=[node])
        dot = serialize_to_dot(graph)
        assert "arrowhead=none" in dot
        assert "penwidth=0.5" in dot
        assert "color=grey" in dot

    def test_tf_topics_not_in_dot_after_extraction(self):
        """/tf and /tf_static topics should not appear as topic nodes after extraction."""
        node_a = _make_node("a", tf_broadcaster=True, tf_listener=True)
        node_b = _make_node("b")
        graph = Graph(
            nodes=[node_a, node_b],
            topics=[
                Topic(
                    name="/cmd_vel",
                    msg_type="geometry_msgs/msg/Twist",
                    publishers=[TopicConnection(node=node_a)],
                    subscribers=[TopicConnection(node=node_b)],
                ),
            ],
        )
        dot = serialize_to_dot(graph)
        # /cmd_vel should be present as a topic node
        assert "/cmd_vel" in dot
        # /tf and /tf_static should NOT be present as topic nodes
        # (they were already extracted before serialization)
        assert '"/tf"' not in dot
        assert '"/tf_static"' not in dot
        # But TF bubbles should be present
        assert "TF Broadcaster" in dot
        assert "TF Listener" in dot


class TestTfInJsonSerialization:
    """Tests for TF role inclusion in JSON serialization."""

    def test_tf_roles_in_json(self):
        """Nodes with TF roles include a tf section in JSON output."""
        node = _make_node("a", tf_listener=True, tf_broadcaster=True, tf_static_broadcaster=True)
        graph = Graph(nodes=[node])
        data = serialize_graph(graph)
        node_data = data["nodes"][0]
        assert node_data["tf"] == {
            "listener": True,
            "broadcaster": True,
            "static_broadcaster": True,
        }

    def test_no_tf_section_when_no_roles(self):
        """Nodes without TF roles omit the tf section in JSON output."""
        node = _make_node("plain")
        graph = Graph(nodes=[node])
        data = serialize_graph(graph)
        node_data = data["nodes"][0]
        assert "tf" not in node_data

    def test_partial_tf_roles_in_json(self):
        """Nodes with some TF roles show all three fields."""
        node = _make_node("a", tf_listener=True)
        graph = Graph(nodes=[node])
        data = serialize_graph(graph)
        node_data = data["nodes"][0]
        assert node_data["tf"] == {
            "listener": True,
            "broadcaster": False,
            "static_broadcaster": False,
        }
