# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os
from dataclasses import dataclass
from inputs import get_gamepad
from typing import Callable
import yaml


@dataclass
class Axes:
    x: float
    y: float
    z: float
    rz: float
    g: float


def default_callback(x: float, y: float, z: float, rz: float, g: float) -> None:
    print(f"[{x}, {y}, {z}], [rx, ry, {rz}], {g}")


class Gamepad:
    def __init__(self, gamepad: str):
        self.callback = default_callback
        file_path = os.path.join(os.path.dirname(__file__), "gamepad.yaml")
        with open(file_path, "r") as stream:
            config = yaml.safe_load(stream)
        self.deadzone = config[gamepad]["deadzone"]
        self.axis_max = config[gamepad]["axis_max"]
        self.zero = config[gamepad]["zero"]
        self.mapping = config[gamepad]["mapping"]

    def set_callback(self, callback: Callable) -> None:
        self.callback = callback

    def run(self) -> None:
        last_known_state = Axes(0, 0, 0, 0, 0)
        while 1:
            events = get_gamepad()
            event = events[0]
            value = event.state - self.zero
            code = event.code
            if code in self.mapping:
                if self.mapping[code] == "gripper_close":
                    last_known_state.g = -1
                elif self.mapping[code] == "gripper_open":
                    last_known_state.g = 1
                elif abs(value) < self.deadzone:
                    setattr(last_known_state, self.mapping[code], 0)
                    continue
                setattr(last_known_state, self.mapping[code], value / self.axis_max)

            if code == "SYN_REPORT":
                self.callback(
                    -last_known_state.y,
                    -last_known_state.x,
                    -last_known_state.z,
                    -last_known_state.rz,
                    last_known_state.g,
                )


if __name__ == "__main__":
    Gamepad("stadia").run()
