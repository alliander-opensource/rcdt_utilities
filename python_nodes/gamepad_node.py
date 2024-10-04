#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move, Grasp
from threading import Thread
from rcdt_utilities.gamepad import Gamepad, GamepadCommand


class GamepadNode(Node):
    def __init__(self):
        self.scaler = 0.5
        super().__init__("rcdt_gamepad")
        self.controlling: str = "none"
        self.msg = TwistStamped()
        self.init_arm_control()
        self.init_base_control()
        self.init_gripper_control()
        self.timer = self.create_timer(1 / 100.0, self.publish)

    def init_arm_control(self) -> None:
        topic = "/servo_node/delta_twist_cmds"
        self.arm_publisher = self.create_publisher(TwistStamped, topic, 10)

    def init_base_control(self) -> None:
        topic = "/diff_drive_controller/cmd_vel"
        self.base_publisher = self.create_publisher(TwistStamped, topic, 10)

    def init_gripper_control(self) -> None:
        self.gripper_state = "close"
        self.move_client = ActionClient(self, Move, "/fr3_gripper/move")
        self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/grasp")

    def publish(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        match self.controlling:
            case "arm":
                self.arm_publisher.publish(self.msg)
            case "base":
                self.base_publisher.publish(self.msg)

    def update(self, command: GamepadCommand) -> None:
        for movement in ["linear", "angular"]:
            command_vector = getattr(command, movement)
            message_vector = getattr(self.msg.twist, movement)
            for axis in ["x", "y", "z"]:
                value = self.scaler * getattr(command_vector, axis)
                deadzone = 0.1
                if abs(value) < deadzone:
                    value = 0.0
                setattr(message_vector, axis, value)
        if command.selected != self.controlling:
            self.update_controlling(command.selected)
        if command.gripper != self.gripper_state:
            self.update_gripper(command.gripper)

    def update_controlling(self, selected: str) -> None:
        match selected:
            case "arm":
                self.msg.header.frame_id = "fr3_link0"
            case "base":
                self.msg.header.frame_id = "base_link"
        self.controlling = selected
        self.get_logger().warn(f"Gamepad control switched to {selected}.")

    def update_gripper(self, command: str) -> None:
        if command == "open":
            client = self.move_client
            goal = Move.Goal()
            goal.width = 0.08
            goal.speed = 0.03
        elif command == "close":
            client = self.grasp_client
            goal = Grasp.Goal()
            goal.width = 0.0
            goal.speed = 0.03
            goal.epsilon.inner = 0.08
            goal.epsilon.outer = 0.08
            goal.force = 100.0
        else:
            return
        client.wait_for_server()
        result: (
            Move.Impl.GetResultService.Response | Grasp.Impl.GetResultService.Response
        ) = client.send_goal(goal)
        if result.result.success:
            self.gripper_state = command
        else:
            self.get_logger().warn("Gripper move did not succeed.")


def main(args: str = None) -> None:
    rclpy.init(args=args)
    gamepad_node = GamepadNode()
    gamepad = Gamepad()
    gamepad.set_callback(gamepad_node.update)
    Thread(target=gamepad.run, daemon=True).start()

    rclpy.spin(gamepad_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
