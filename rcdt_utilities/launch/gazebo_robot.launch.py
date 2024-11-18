# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_file_path

default_world = get_file_path(
    "rcdt_utilities", ["sdf"], "empty_world_with_camera_plugin.xml.sdf"
)

world_arg = LaunchArgument("world", default_world)


def launch_setup(context: LaunchContext) -> List:
    gazebo = IncludeLaunchDescription(
        get_file_path("ros_gz_sim", ["launch"], "gz_sim.launch.py"),
        launch_arguments={"gz_args": f" -r {world_arg.value(context)}"}.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        output="screen",
    )

    sync_clock = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="sync_clock",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    return [
        gazebo,
        spawn_robot,
        sync_clock
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            world_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
