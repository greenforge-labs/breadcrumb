cmake_minimum_required(VERSION 3.8)
project(breadcrumb)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION bin)

ament_package()
