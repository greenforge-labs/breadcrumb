# breadcrumb_example

A comprehensive example package demonstrating the integration of **cake**, **clingwrap**, and **breadcrumb** for ROS2 development and static analysis.

## Overview

This package showcases:
- **Cake**: Automatic code generation from YAML interface definitions
- **Clingwrap**: Clean, Pythonic launch file syntax with static analysis
- **Breadcrumb**: Graph visualization and analysis of ROS2 systems

## Package Structure

```
breadcrumb_example/
├── nodes/                      # Cake node definitions
│   ├── sensor_driver/          # C++ node - publishes sensor data
│   ├── data_processor/         # C++ composable node - processes data
│   ├── monitor_node/           # Python node - monitors system
│   └── controller/             # C++ node - action server
├── interfaces/                 # External node interface definitions
│   └── external_camera.yaml    # Example third-party node interface
├── launch/                     # Clingwrap launch files
│   ├── sensors.launch.py       # Sensor subsystem
│   ├── processing.launch.py    # Processing subsystem
│   └── full_system.launch.py   # Complete multi-robot system
├── config/
│   └── params.yaml             # Example parameters
├── CMakeLists.txt              # Just 3 lines with cake_auto_package()!
└── package.xml
```

## Nodes

### sensor_driver (C++)
- **Publishers**: `sensor_data` (Temperature), `sensor_status` (String)
- **Parameters**: `sensor_id`, `update_rate`, `frame_id`
- **Description**: Simulates a temperature sensor with configurable update rate

### data_processor (C++ Composable)
- **Subscribers**: `sensor_data` (Temperature)
- **Publishers**: `processed_data` (Temperature), `alerts` (String)
- **Services**: `get_statistics` (Trigger)
- **Parameters**: `processing_enabled`, `threshold`
- **Description**: Processes sensor data, tracks statistics, generates alerts

### monitor_node (Python)
- **Subscribers**: `processed_data` (Temperature), `alerts` (String)
- **Service Clients**: `get_statistics` (Trigger)
- **Parameters**: `log_interval`
- **Description**: Monitors system activity and periodically requests statistics

### controller (C++)
- **Publishers**: `cmd_vel` (Twist)
- **Action Servers**: `navigate_to_pose` (NavigateToPose)
- **Action Clients**: `compute_path_to_pose` (ComputePathToPose)
- **Parameters**: `max_speed`
- **Description**: Navigation controller with action server

### external_camera (Interface Only)
- **Package**: `vendor_camera_package` (hypothetical)
- **Publishers**: `image_raw` (Image), `camera_info` (CameraInfo)
- **Services**: `set_camera_info`, `start_capture`, `stop_capture`
- **Description**: Example of defining an interface for an external node you don't control

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select breadcrumb_example
source install/setup.bash
```

During the build, cake will:
1. Generate C++ and Python interface code
2. Generate parameter libraries
3. Build executables and libraries
4. **Install interface.yaml files to `share/breadcrumb_example/interfaces/`**
5. Register C++ nodes as composable components

## Running

### Individual Subsystems

```bash
# Launch sensor subsystem
ros2 launch breadcrumb_example sensors.launch.py

# Launch processing subsystem
ros2 launch breadcrumb_example processing.launch.py

# Launch full multi-robot system
ros2 launch breadcrumb_example full_system.launch.py
```

### Individual Nodes

```bash
# Run sensor driver
ros2 run breadcrumb_example sensor_driver

# Run as composable component
ros2 component standalone breadcrumb_example breadcrumb_example::SensorDriver

# Run Python monitor node
ros2 run breadcrumb_example monitor_node
```

## Analyzing with Breadcrumb

### Basic Analysis

Analyze the sensor subsystem:
```bash
breadcrumb launch/sensors.launch.py
```

Output:
```
================================================================================
Graph Analysis Complete
================================================================================

Nodes: 3
Topics: 3
Services: 0
Actions: 0
```

### Generate JSON Output

```bash
breadcrumb launch/full_system.launch.py -o system_graph.json
```

### Generate GraphViz Visualization

#### Full System Graph

```bash
breadcrumb launch/full_system.launch.py -o graph.dot --graph-type full_system
dot -Tpng graph_full_system.dot -o graph.png
```

#### Grouped by Namespace

This is particularly useful for multi-robot systems:

```bash
breadcrumb launch/full_system.launch.py -o graph.dot --graph-type grouped_by_namespace
```

This generates:
- `graph_toplevel.dot` - Shows robot1, robot2, and shared namespaces with inter-namespace communication
- `graph_group_robot1.dot` - Detailed view of robot1's nodes and connections
- `graph_group_robot2.dot` - Detailed view of robot2's nodes and connections
- `graph_group_diagnostics.dot` - Diagnostics namespace (from topic relay)

Render all graphs:
```bash
for file in graph_*.dot; do
    dot -Tpng "$file" -o "${file%.dot}.png"
done
```

#### Both Views

```bash
breadcrumb launch/full_system.launch.py -o graph.dot --graph-type grouped_and_full_system
```

### Analyzing Multiple Launch Files

Breadcrumb can analyze multiple launch files together:

```bash
breadcrumb launch/sensors.launch.py launch/processing.launch.py -o combined.json
```

## Expected Graph Structure

### Full System (full_system.launch.py)

The complete system creates a multi-robot setup:

**robot1 namespace:**
- sensor_driver → sensor_data → data_processor
- data_processor → processed_data → monitor_node
- data_processor → alerts → monitor_node
- monitor_node → get_statistics (service client) → data_processor
- controller → cmd_vel

**robot2 namespace:**
- Same structure as robot1, completely isolated

**Diagnostics namespace:**
- Topic relays from each robot's sensor_status

**Composable Nodes:**
- data_processor runs in a container (processing_container)

**Topic Tools:**
- Relays: `/sensor_status` → `/diagnostics/sensor_status`
- Throttles: `/sensor_data/throttled/hz_1_0` (1 Hz throttled version)

## Key Features Demonstrated

### Cake Features
- ✅ C++ nodes with automatic interface generation
- ✅ Python nodes with automatic interface generation
- ✅ Composable nodes with component registration
- ✅ Parameters with validation
- ✅ Publishers, subscribers, services, actions
- ✅ Context-based architecture
- ✅ Automatic interface.yaml installation

### Clingwrap Features
- ✅ Clean launch file syntax
- ✅ Namespace context managers
- ✅ Composable node containers
- ✅ Topic relay and throttle
- ✅ Launch file inclusion with arguments
- ✅ Static analysis capability

### Breadcrumb Features
- ✅ Launch file parsing and node extraction
- ✅ Interface discovery from installed YAML files
- ✅ Multi-namespace system analysis
- ✅ External node interface definitions
- ✅ JSON export for programmatic analysis
- ✅ GraphViz visualization (full system and grouped)
- ✅ Launch file inclusion tracking

## Troubleshooting

### "Could not find node interface YAML"

Make sure you've built the package:
```bash
colcon build --packages-select breadcrumb_example
source install/setup.bash
```

Verify interfaces are installed:
```bash
ls $(ros2 pkg prefix breadcrumb_example)/share/breadcrumb_example/interfaces/
```

### Breadcrumb shows fewer nodes than expected

Check that your launch file uses clingwrap's `LaunchBuilder`. Standard ROS2 launch files cannot be statically analyzed.

### Graph visualization is cluttered

Use grouped mode to see namespace-separated views:
```bash
breadcrumb launch/full_system.launch.py -o graph.dot --graph-type grouped_by_namespace
```

## Learning Resources

- **Cake**: See generated files in `build/breadcrumb_example/` after building
- **Clingwrap**: Run `python3 -c "import clingwrap; help(clingwrap.LaunchBuilder)"`
- **Breadcrumb**: Run `breadcrumb --help`

## License

Apache 2.0
