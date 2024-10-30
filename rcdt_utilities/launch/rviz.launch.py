# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities_py.launch_utils import LaunchArgument, get_file_path

display_config_general = get_file_path("rcdt_utilities", ["rviz"], "general.rviz")

rviz_frame_arg = LaunchArgument("rviz_frame", "")


def launch_setup(context: LaunchContext) -> None:
    arguments = ["--display-config", display_config_general]

    rviz_frame = rviz_frame_arg.value(context)
    if rviz_frame != "":
        arguments.extend(["-f", rviz_frame])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=arguments,
    )

    return [rviz]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            rviz_frame_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
