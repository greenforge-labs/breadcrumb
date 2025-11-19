cmake_minimum_required(VERSION 3.22)
project(breadcrumb_example_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME} "action/TrackPosition.action" DEPENDENCIES action_msgs)

ament_package()
