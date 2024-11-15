# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node, SetParameter
from rcdt_utilities.launch_utils import (
    get_file_path,
    get_robot_description,
)

pkg_share = get_package_share_directory("rcdt_realsense")


def launch_setup(_context: LaunchContext) -> None:
    xacro_path = get_file_path(
        "rcdt_utilities", ["urdf"], "rcdt_realsense_d435.urdf.xacro"
    )
    xacro_arguments = {"simulation": "true", "parent": "world"}
    robot_description = get_robot_description(xacro_path, xacro_arguments)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    robot = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "gazebo_robot.launch.py"),
        launch_arguments={
            "world": "/home/yuripro/husarion_ws/src/husarion_gz_worlds/worlds/husarion_world.sdf"
        }.items(),
    )

    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=[
            os.path.join(pkg_share, "worlds"),
            ":" + str(Path(pkg_share).parent.resolve()),
        ],
    )

    rgb_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/some_name/color/image_raw",
            # "/camera/camera/color/image_raw"
        ],
        output="screen",
    )

    # sensor_msgs/msg/PointCloud2

    # ignition::msgs::PointCloudPacked

    depth_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/ns/namespace/depth/image_rect_raw/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
    )

    rviz = IncludeLaunchDescription(
        get_file_path("rcdt_utilities", ["launch"], "rviz.launch.py"),
        launch_arguments={
            # TODO: placeholder
            "rviz_frame": "/world",
            "rviz_display_config": get_file_path(
                "rcdt_utilities", ["rviz"], "realsense.rviz"
            ),
        }.items(),
    )

    return [
        SetParameter(name="use_sim_time", value=True),
        ign_resource_path,
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
