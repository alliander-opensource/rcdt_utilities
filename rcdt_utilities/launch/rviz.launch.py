# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

display_config_general = get_file_path("rcdt_utilities", ["rviz"], "general.rviz")

rviz_frame_arg = LaunchArgument("rviz_frame", "world")
rviz_display_config = LaunchArgument("rviz_display_config", display_config_general)


def launch_setup(context: LaunchContext) -> None:
    arguments = ["--display-config", rviz_display_config.value(context)]

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
            rviz_display_config.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
