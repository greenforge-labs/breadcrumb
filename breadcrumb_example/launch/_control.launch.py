import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("control"):
        # Launch the LQR controller with topic remapping
        l.node(
            "breadcrumb_example",
            "cartpole_controller",
            remappings={
                "joint_states": "/hardware/joint_states",
                "force": "/hardware/request/force",
            },
        )

    return l
