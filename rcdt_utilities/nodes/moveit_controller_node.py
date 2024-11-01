#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor

from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState
from moveit.core.robot_model import RobotModel, JointModelGroup
from moveit.core.planning_interface import MotionPlanResponse
from rcdt_utilities_msgs.action import Moveit
from rcdt_utilities_msgs.srv import AddMarker

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


class MoveitControllerClient(Node):
    def __init__(self) -> None:
        super().__init__("moveit_controller_client")
        self.add_marker = self.create_client(AddMarker, "rviz_controller/add_marker")
        self.clear_markers = self.create_client(Trigger, "rviz_controller/clear_all")


class MoveitControllerNode(Node):
    def __init__(self, group: str, client: MoveitControllerClient) -> None:
        name = "moveit_controller"
        super().__init__(name)

        self.client = client

        self.robot = MoveItPy(node_name="moveit_py")
        robot_model: RobotModel = self.robot.get_robot_model()
        joint_model_group: JointModelGroup = robot_model.get_joint_model_group(group)

        end_effectors: List[JointModelGroup] = robot_model.end_effectors
        if len(end_effectors) == 0:
            self.get_logger().error("No end-effector was found. Exiting...")
            return
        end_effector = end_effectors[0]
        if len(end_effectors) > 1:
            self.get_logger().warn(
                f"Multiple end-effector defined. First one ('{end_effector.name}') is used."
            )
        self.ee_link = end_effector.link_model_names[0]

        self.links: List[str] = joint_model_group.link_model_names
        self.planner: PlanningComponent = self.robot.get_planning_component(group)

        self.server = ActionServer(self, Moveit, name, self.callback)

    def update_state(self) -> None:
        self.planner.set_start_state_to_current_state()
        self.state: RobotState = self.planner.get_start_state()

    def callback(self, goal_handle: ServerGoalHandle) -> None:
        goal: Moveit.Goal = goal_handle.request

        self.update_state()
        feedback = Moveit.Feedback()
        feedback.start_pose = self.state.get_pose(self.ee_link)
        goal_handle.publish_feedback(feedback)

        success = self.move_to_pose(goal.goal_pose)
        goal_handle.succeed() if success else goal_handle.abort()

        self.update_state()
        result = Moveit.Result()
        result.end_pose = self.state.get_pose(self.ee_link)

        return result

    def mark_pose(self, pose: PoseStamped) -> None:
        if not (
            self.client.clear_markers.wait_for_service(1)
            and self.client.add_marker.wait_for_service(1)
        ):
            self.get_logger().warn("Rviz services not available, marking pose skipped.")
            return
        request = Trigger.Request()
        self.client.clear_markers.call(request)
        request = AddMarker.Request()
        request.marker_pose = pose
        self.client.add_marker.call(request)

    def plan_and_execute(self) -> bool:
        plan: MotionPlanResponse = self.planner.plan()
        if not plan:
            self.get_logger().error("No motion plan was found. Aborting...")
            return False

        self.robot.execute(plan.trajectory, controllers=[])
        return True

    def move_to_pose(self, pose: PoseStamped) -> bool:
        if pose.header.frame_id not in self.links:
            self.get_logger().error(
                f"frame_id '{pose.header.frame_id}' not one of links: {self.links}"
            )
            return False
        self.planner.set_goal_state(pose_stamped_msg=pose, pose_link=self.ee_link)

        self.mark_pose(pose)
        return self.plan_and_execute()


def main(args: str = None) -> None:
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()

    moveit_controller_client = MoveitControllerClient()
    executor.add_node(moveit_controller_client)

    moveit_controller = MoveitControllerNode("fr3_arm", moveit_controller_client)
    executor.add_node(moveit_controller)

    executor.spin()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
