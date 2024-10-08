# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.5)

project(rcdt_utilities)

find_package(ament_cmake REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install project launch files
install(
  DIRECTORY
    launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  nodes/joy_to_twist_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
