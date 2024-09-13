# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from dataclasses import dataclass
from inputs import get_gamepad
from typing import Callable


@dataclass
class Axes:
    x: float
    y: float
    rx: float
    ry: float


mapping = {
    "ABS_X": "x",
    "ABS_RX": "rx",
    "ABS_Y": "y",
    "ABS_RY": "ry",
}


def default_callback(x: float, y: float, z: float) -> None:
    print(x, y, z)


class Gamepad:
    def __init__(self):
        self.callback = default_callback
        self.deadzone = 3000
        self.axis_max = 32768

    def set_callback(self, callback: Callable) -> None:
        self.callback = callback

    def run(self) -> None:
        last_known_state = Axes(0, 0, 0, 0)
        while 1:
            events = get_gamepad()
            event = events[0]
            if event.code in mapping:
                if abs(event.state) < self.deadzone:
                    setattr(last_known_state, mapping[event.code], 0)
                    continue
                setattr(
                    last_known_state, mapping[event.code], event.state / self.axis_max
                )

            if event.code == "SYN_REPORT":
                self.callback(
                    -last_known_state.y,
                    -max([last_known_state.x, last_known_state.rx], key=abs),
                    -last_known_state.ry,
                )


if __name__ == "__main__":
    Gamepad().run()
