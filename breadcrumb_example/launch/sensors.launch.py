"""
Launch file for sensor subsystem.

Demonstrates:
- Basic nodes with parameters
- Topic relay and throttle
- Remappings
"""

import clingwrap as cw


def generate_launch_description():
    l = cw.LaunchBuilder()

    # Declare arguments
    sensor_rate = l.declare_arg("sensor_rate", default_value="10.0")

    # Launch sensor driver with parameters
    l.node(
        "breadcrumb_example",
        "sensor_driver",
        name="main_sensor",
        parameters={
            "sensor_id": "main_001",
            "update_rate": sensor_rate,
            "frame_id": "sensor_link",
        },
        remappings={
            "sensor_status": "diagnostics/sensor_status",
        },
    )

    # Throttle sensor data to 1 Hz for logging (demonstrates throttle)
    l.topic_throttle_hz("sensor_data", rate=1.0, include_hz_in_output_topic=True)

    return l
