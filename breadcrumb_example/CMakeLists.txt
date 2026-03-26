cmake_minimum_required(VERSION 3.22)
project(breadcrumb_example)

find_package(jig REQUIRED)
jig_auto_package(INSTALL_TO_SHARE urdf rviz web_templates)
