"""
Launch file for data processing subsystem.

Demonstrates:
- Composable nodes in containers
- Different container types
"""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Declare arguments
    threshold = l.declare_arg("threshold", default_value="25.0")

    # Create a composable node container with data processor
    with l.composable_node_container(name="processing_container"):
        l.composable_node(
            "breadcrumb_example",
            "breadcrumb_example::DataProcessor",
            name="data_processor",
            parameters={"processing_enabled": True, "threshold": threshold},
        )

    # Launch monitor node as regular node
    l.node("breadcrumb_example", "monitor_node", name="system_monitor", parameters={"log_interval": 5.0})

    return l
