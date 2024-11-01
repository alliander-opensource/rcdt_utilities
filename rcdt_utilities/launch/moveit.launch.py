# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import (
    LaunchArgument,
    get_yaml,
    get_file_path,
    get_moveit_parameters,
)

moveit_arg = LaunchArgument("moveit", "off", ["classic", "servo", "off"])
moveit_config_package_arg = LaunchArgument(
    "moveit_config_package", "rcdt_franka_moveit_config"
)
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")


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

    servo_params_package = servo_params_package_arg.value(context)
    file = get_file_path(servo_params_package, ["config"], "servo_params.yaml")
    servo_config = get_yaml(file)
    servo_params = {"moveit_servo": servo_config}
    moveit_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        parameters=[
            servo_params,
            moveit_config,
        ],
    )

    set_servo_command_type = ExecuteProcess(
        cmd=[
            [
                "ros2 service call ",
                "/servo_node/switch_command_type ",
                "moveit_msgs/srv/ServoCommandType ",
                "'{command_type: 1}'",
            ]
        ],
        shell=True,
    )

    skip = LaunchDescriptionEntity()
    return [
        moveit_classic if moveit_arg.value(context) == "classic" else skip,
        static_transform_publisher if moveit_arg.value(context) != "off" else skip,
        moveit_servo if moveit_arg.value(context) == "servo" else skip,
        set_servo_command_type if moveit_arg.value(context) == "servo" else skip,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            moveit_arg.declaration,
            moveit_config_package_arg.declaration,
            servo_params_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
