# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    get_file_path,
    get_robot_description,
)


def launch_setup(_context: LaunchContext) -> None:
    xacro_path = get_file_path(
        "rcdt_utilities", ["urdf"], "rcdt_realsense_d435.urdf.xacro"
    )
    robot_description = get_robot_description(xacro_path)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py")
    )

    rgb_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera/color/image_raw",
        ],
        output="screen",
    )

    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/depth/image_rect_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            "rviz_display_config": get_file_path(
                "rcdt_utilities", ["rviz"], "realsense.rviz"
            )
        }.items(),
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        robot_state_publisher,
        robot,
        rgb_bridge,
        depth_bridge,
        rviz,
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            # simulation_arg.declaration,
            OpaqueFunction(function=launch_setup),
        ]
    )
