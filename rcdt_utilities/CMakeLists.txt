# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_utilities)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install project files
install(
  DIRECTORY
  launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install nodes
install(PROGRAMS
  nodes/joy_to_twist_node.py
  nodes/moveit_action_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Build actions:
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Moveit.action"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install python package
ament_python_install_package(${PROJECT_NAME}_py)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
