import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    with l.namespace("control"):
        # Launch the LQR controller with topic remapping
        l.node(
            "breadcrumb_example",
            "cartpole_controller",
            parameters={
                "k1": -3.16,
                "k2": -5.64,
                "k3": -56.84,
                "k4": -10.86,
            },
            remappings={
                "joint_states": "/hardware/joint_states",
                "force": "/hardware/request/force",
            },
        )

    return l
