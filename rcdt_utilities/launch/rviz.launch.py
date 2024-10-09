# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_moveit_parameters,
    get_file_path,
)

display_config_general = get_file_path("rcdt_utilities", ["rviz"], "general.rviz")
display_config_moveit = get_file_path("rcdt_utilities", ["rviz"], "moveit.rviz")

rviz_frame_arg = LaunchArgument("rviz_frame", "")
rviz_load_moveit_arg = LaunchArgument("rviz_load_moveit", False, [True, False])
rviz_load_moveit_robot_arg = LaunchArgument("rviz_load_moveit_robot", "")
rviz_load_moveit_package_arg = LaunchArgument("rviz_load_moveit_package", "")


def launch_setup(context: LaunchContext) -> None:
    arguments = []

    rviz_frame = rviz_frame_arg.value(context)
    if rviz_frame != "":
        arguments.extend(["-f", rviz_frame])

    if rviz_load_moveit_arg.value(context):
        robot_name = rviz_load_moveit_robot_arg.value(context)
        package_name = rviz_load_moveit_package_arg.value(context)
        moveit_parameters = get_moveit_parameters(robot_name, package_name)
        parameters = [moveit_parameters]
        arguments.extend(["--display-config", display_config_moveit])
    else:
        parameters = []
        arguments.extend(["--display-config", display_config_general])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=arguments,
        parameters=parameters,
    )

    return [rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            rviz_frame_arg.declaration,
            rviz_load_moveit_arg.declaration,
            rviz_load_moveit_robot_arg.declaration,
            rviz_load_moveit_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
