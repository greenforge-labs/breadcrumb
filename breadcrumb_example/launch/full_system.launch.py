"""
Full system launch file demonstrating multi-robot setup.

Demonstrates:
- Launch file inclusion
- Multiple namespaces
- External node interfaces
- Complete system integration
"""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # ===== Robot 1 =====
    with l.namespace("robot1"):
        # Include sensor subsystem
        l.include_launch_py(
            package="breadcrumb_example", launch_file="sensors.launch.py", launch_arguments={"sensor_rate": "15.0"}
        )

        # Include processing subsystem
        l.include_launch_py(
            package="breadcrumb_example", launch_file="processing.launch.py", launch_arguments={"threshold": "23.0"}
        )

        # Launch controller
        l.node("breadcrumb_example", "controller", name="controller", parameters={"max_speed": 1.5})

    # ===== Robot 2 =====
    with l.namespace("robot2"):
        # Include sensor subsystem with different parameters
        l.include_launch_py(
            package="breadcrumb_example", launch_file="sensors.launch.py", launch_arguments={"sensor_rate": "20.0"}
        )

        # Include processing subsystem with different parameters
        l.include_launch_py(
            package="breadcrumb_example", launch_file="processing.launch.py", launch_arguments={"threshold": "27.0"}
        )

        # Launch controller
        l.node("breadcrumb_example", "controller", name="controller", parameters={"max_speed": 1.0})

    # ===== Shared Resources =====
    # Launch an external camera node (uses interface from interfaces/ folder)
    # Note: This is a placeholder - the actual node doesn't exist,
    # but breadcrumb will use the interface definition from interfaces/external_camera.yaml
    with l.namespace("shared"):
        # In a real system, you would launch the actual vendor camera node here
        # For demonstration purposes, we just document the interface
        # l.node("vendor_camera_package", "camera_driver", name="shared_camera")
        pass

    return l
