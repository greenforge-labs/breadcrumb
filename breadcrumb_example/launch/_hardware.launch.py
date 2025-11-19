import clingwrap as cw

from launch import substitutions as sub


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Get path to the cartpole URDF
    urdf_path = cw.pkg_file("breadcrumb_example", "urdf", "cartpole.urdf")

    # Read URDF file contents
    robot_description = sub.Command(["cat ", urdf_path])

    with l.namespace("hardware"):
        # Launch robot_state_publisher with the URDF
        l.node(
            "robot_state_publisher",
            parameters={"robot_description": cw.as_str_param(robot_description)},
        )

        # Launch the cartpole simulator
        l.node(
            "breadcrumb_example",
            "cartpole_simulator",
            remappings={
                "requested_force": "request/force",
            },
        )

    # Launch RViz2
    l.node("rviz2", arguments=["-d", cw.pkg_file("breadcrumb_example", "rviz", "cartpole_simulator.rviz")])

    return l
