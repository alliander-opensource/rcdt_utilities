# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription, LaunchContext, LaunchDescriptionEntity
from launch.actions import OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from rcdt_utilities.launch_utils import LaunchArgument, get_yaml, get_file_path
from moveit_configs_utils import MoveItConfigsBuilder

SKIP = LaunchDescriptionEntity()

use_sim_arg = LaunchArgument("simulation", True, [True, False])
moveit_mode_arg = LaunchArgument("moveit", "off", ["node", "rviz", "servo", "off"])
moveit_config_package_arg = LaunchArgument(
    "moveit_config_package", "rcdt_franka_moveit_config"
)
servo_params_package_arg = LaunchArgument("servo_params_package", "rcdt_franka")


def launch_setup(context: LaunchContext) -> None:
    use_sim = use_sim_arg.value(context)
    moveit_mode = moveit_mode_arg.value(context)
    config_package = moveit_config_package_arg.value(context)
    servo_params_package = servo_params_package_arg.value(context)

    # Create moveit config dictionary:
    moveit_config = MoveItConfigsBuilder(robot_name="fr3", package_name=config_package)
    if moveit_mode == "node":
        moveit_config.trajectory_execution(
            get_file_path(config_package, ["config"], "moveit_controllers.yaml")
        )
        moveit_config.moveit_cpp(
            get_file_path(config_package, ["config"], "planning_pipeline.yaml")
        )
    moveit_config = moveit_config.to_dict()

    # Moveit as node:
    moveit_node = Node(
        package="rcdt_utilities",
        executable="moveit_controller_node.py",
        parameters=[moveit_config, {"use_sim_time": use_sim}],
    )

    # Moveit in rviz:
    moveit_rviz = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[moveit_config],
    )

    display_config_moveit = get_file_path("rcdt_utilities", ["rviz"], "moveit.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["--display-config", display_config_moveit],
        parameters=[moveit_config],
    )

    # Moveit servo:
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

    # Select moveit:
    moveit_selector = {
        "node": moveit_node,
        "rviz": moveit_rviz,
        "servo": moveit_servo,
        "off": SKIP,
    }
    moveit = moveit_selector[moveit_mode]

    return [
        moveit,
        rviz if moveit_mode == "rviz" else SKIP,
        set_servo_command_type if moveit_mode == "servo" else SKIP,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            moveit_mode_arg.declaration,
            moveit_config_package_arg.declaration,
            servo_params_package_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
