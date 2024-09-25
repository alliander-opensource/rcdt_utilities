#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import TwistStamped
from franka_msgs.action import Move, Grasp
from threading import Thread
from rcdt_utilities.gamepad import Gamepad, GamepadCommand, Vec3

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
        self.init_tf_listener()
        self.timer = self.create_timer(1 / 10, self.update_position)
        self.timer = self.create_timer(1 / 100.0, self.publish)

    def init_tf_listener(self) -> None:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_position = None
        self.position = Vec3()

    def update_position(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                "fr3_link0", "fr3_hand", rclpy.time.Time()
            )
            position = transform.transform.translation

        except Exception:
            return

        for axis in ["x", "y", "z"]:
            setattr(self.position, axis, getattr(position, axis))
        if self.goal_position is None:
            self.goal_position = Vec3()
            for axis in ["x", "y", "z"]:
                setattr(self.goal_position, axis, getattr(self.position, axis))

    def publish(self) -> None:
        self.define_error()
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.msg)

    def define_error(self) -> None:
        if self.goal_position is None:
            return
        for axis in ["x", "y", "z"]:
            sign = -1 if axis in ["y", "z"] else 1
            error = getattr(self.goal_position, axis) - getattr(self.position, axis)
            error *= sign
            gain_p = 10
            velocity = float(max(min(gain_p * error, 1), -1))
            setattr(self.msg.twist.linear, axis, velocity)

    def update(self, command: GamepadCommand) -> None:
        self.update_linear(command.linear)
        self.update_angular(command.angular)
        if command.gripper != self.gripper_state:
            self.update_gripper(command.gripper)

    def update_linear(self, command: Vec3) -> None:
        if self.goal_position is None:
            return
        scaler = 0.001
        for axis in ["x", "y", "z"]:
            value = getattr(command, axis)
            deadzone = 0.1
            if abs(value) < deadzone:
                value = 0.0
            new_value = getattr(self.goal_position, axis) + scaler * value
            setattr(self.goal_position, axis, new_value)

    def update_angular(self, command: Vec3) -> None:
        for axis in ["x", "y", "z"]:
            value = self.scaler * getattr(command, axis)
            deadzone = 0.1
            if abs(value) < deadzone:
                value = 0.0
            setattr(self.msg.twist.angular, axis, value)

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
