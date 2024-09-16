#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move, Grasp
from threading import Thread
from rcdt_utilities.gamepad import Gamepad
from rcdt_utilities.virtual_gamepad import VirtualGamepad


@dataclass
class Vec:
    x: float
    y: float
    z: float
    rz: float


class GamepadNode(Node):
    def __init__(self):
        self.scale_factor = 0.5
        super().__init__("rcdt_gamepad")
        self.declare_parameter("gamepad", "virtual")
        self.topic = "/servo_node/delta_twist_cmds"
        self.frame_id = "fr3_link0"
        self.publisher = self.create_publisher(TwistStamped, self.topic, 10)
        self.move_client = ActionClient(self, Move, "/fr3_gripper/move")
        self.grasp_client = ActionClient(self, Grasp, "/fr3_gripper/grasp")
        self.vector = Vec(0, 0, 0, 0)
        self.gripper_state = -1
        self.timer = self.create_timer(1 / 100.0, self.publish)

    def publish(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(self.vector.x)
        msg.twist.linear.y = float(self.vector.y)
        msg.twist.linear.z = float(self.vector.z)
        msg.twist.angular.z = float(self.vector.rz)
        self.publisher.publish(msg)

    def update(self, x: float, y: float, z: float, rz: float, g: float = 0) -> None:
        self.update_vector(x, y, z, rz)
        self.update_gripper(g)

    def update_vector(self, x: float, y: float, z: float, rz: float) -> None:
        self.vector = Vec(
            x * self.scale_factor,
            y * self.scale_factor,
            z * self.scale_factor,
            rz * self.scale_factor,
        )

    def update_gripper(self, g: float) -> None:
        if g == 1 and self.gripper_state != 1:
            goal = Move.Goal()
            goal.width = 0.04
            self.move_client.wait_for_server()
            if self.move_client.send_goal_async(goal):
                self.gripper_state = 1
        elif g == -1 and self.gripper_state != -1:
            goal = Grasp.Goal()
            goal.width = 0.0
            goal.force = 100.0
            self.grasp_client.wait_for_server()
            if self.grasp_client.send_goal_async(goal):
                self.gripper_state = -1


def main(args: str = None) -> None:
    rclpy.init(args=args)
    gamepad_node = GamepadNode()
    name = gamepad_node.get_parameter("gamepad").get_parameter_value().string_value
    gamepad = VirtualGamepad() if name == "virtual" else Gamepad(name)
    gamepad.set_callback(gamepad_node.update)
    Thread(target=gamepad.run, daemon=True).start()

    rclpy.spin(gamepad_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
