# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_yaml,
    get_file_path,
    get_moveit_parameters,
)

servo_config = get_yaml(get_file_path("rcdt_franka", ["config"], "servo_params.yaml"))

moveit_arg = LaunchArgument("moveit", "off", ["classic", "servo", "off"])
moveit_config_package_arg = LaunchArgument(
    "moveit_config_package", "rcdt_franka_moveit_config"
)


def launch_setup(context: LaunchContext) -> None:
    moveit_config_package = moveit_config_package_arg.value(context)
    moveit_config = get_moveit_parameters("fr3", moveit_config_package)

    moveit_classic = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
    )

    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_base",
        arguments=["--frame-id", "world", "--child-frame-id", "base"],
    )

    servo_params = {"moveit_servo": servo_config}
    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config,
        ],
    )

    gamepad = Node(
        package="rcdt_utilities",
        executable="gamepad_node.py",
    )

    skip = LaunchDescriptionEntity()
    return [
        moveit_classic if moveit_arg.value(context) == "classic" else skip,
        static_transform_publisher if moveit_arg.value(context) == "classic" else skip,
        moveit_servo if moveit_arg.value(context) == "servo" else skip,
        gamepad if moveit_arg.value(context) == "servo" else skip,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            moveit_arg.declaration,
            moveit_config_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
