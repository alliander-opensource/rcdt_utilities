#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.logging import get_logger
from rclpy.impl.rcutils_logger import RcutilsLogger

import time

from moveit.planning import MoveItPy, PlanningComponent
from moveit.core.robot_state import RobotState


def plan_and_execute(
    robot: MoveItPy,
    planning_component: PlanningComponent,
    logger: RcutilsLogger,
) -> None:
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")


def main() -> None:
    rclpy.init()
    logger = get_logger("moveit_py.pose_goal")

    fr3 = MoveItPy(node_name="moveit_py")
    fr3_arm: PlanningComponent = fr3.get_planning_component("fr3_arm")
    logger.warn("MoveItPy instance created")

    time.sleep(10)
    logger.warn("MoveItPy starting...")

    robot_model = fr3.get_robot_model()
    robot_state = RobotState(robot_model)

    robot_state.set_to_random_positions()
    fr3_arm.set_start_state_to_current_state()
    fr3_arm.set_goal_state(robot_state=robot_state)
    plan_and_execute(fr3, fr3_arm, logger)


if __name__ == "__main__":
    main()
