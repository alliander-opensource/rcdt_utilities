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
        self.topic = "/servo_node/delta_twist_cmds"
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)
        self.move_client = ActionClient(self, Move, "/fr3_gripper/move")
        self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/grasp")
        self.msg = TwistStamped()
        self.msg.header.frame_id = "fr3_link0"
        self.gripper_state = "close"
        self.timer = self.create_timer(1 / 100.0, self.publish)

    def publish(self) -> None:
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

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
        if command.gripper != self.gripper_state:
            self.update_gripper(command.gripper)

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
            goal.force = 100.0
        else:
            return
        client.wait_for_server()
        if client.send_goal_async(goal):
            self.gripper_state = command


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
