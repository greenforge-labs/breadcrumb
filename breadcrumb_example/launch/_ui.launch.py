import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Launch the web UI node
    l.node(
        "breadcrumb_example",
        "cartpole_ui",
        parameters={
            "webserver_port": 3000,
            "controller_node_name": "control.cartpole_controller",
        },
        remappings={
            "joint_states": "/hardware/joint_states",
            "enable_controller": "/control/enable",
            "reset_simulator": "/hardware/reset",
            **cw.remap_action("track_position", "/control/track_position"),
        },
    )

    return l
