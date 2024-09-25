# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import os
import yaml
import pygame
from typing import Callable
from dataclasses import dataclass
import dearpygui.dearpygui as dpg
import dearpygui_grid as dpg_grid


@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class GamepadCommand:
    linear = Vec3()
    angular = Vec3()
    gripper = "closed"


def default_callback(command: GamepadCommand) -> None:
    for movement in ["linear", "angular"]:
        print(f"{movement}: ", end="")
        vector = getattr(command, movement)
        for axis in ["x", "y", "z"]:
            print(f"{axis}: {round(getattr(vector, axis), 1)}, ", end="")
    print(f"gripper: {command.gripper}", end="")
    print()


class Gamepad:
    def __init__(self):
        self.callback = default_callback
        self.command = GamepadCommand()
        self.initialize_gamepads()

    def set_callback(self, callback: Callable) -> None:
        self.callback = callback

    def initialize_gamepads(self) -> None:
        pygame.init()
        config_path = os.path.join(os.path.dirname(__file__), "gamepad.yaml")
        with open(config_path, "r") as stream:
            self.config = yaml.safe_load(stream)
        self.joysticks = {}
        for i in range(0, pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(i)
            if joystick.get_name() in self.config:
                self.joysticks[i] = joystick
                joystick.init()

    def run(self) -> None:
        if not len(self.joysticks):
            self.run_virtual_gamepad()
            return
        clock = pygame.time.Clock()
        while True:
            clock.tick(60)
            for event in pygame.event.get():
                if "joy" in event.dict:
                    self.handle_event(event.dict)

    def run_virtual_gamepad(self) -> None:
        virtual_gamepad = VirtualGamepad()
        virtual_gamepad.set_callback(self.callback)
        virtual_gamepad.run()

    def handle_event(self, event: dict) -> None:
        if "axis" in event:
            self.handle_axis(event)
        elif "button" in event:
            self.handle_button(event)
        else:
            return
        self.callback(self.command)

    def handle_button(self, event: dict) -> None:
        joystick: pygame.joystick.JoystickType = self.joysticks[event["joy"]]
        mapping = self.config[joystick.get_name()]
        buttons = mapping["buttons"]
        button = event["button"]
        if button not in buttons:
            return
        if buttons[button] == "gripper_open":
            self.command.gripper = "open"
        elif buttons[button] == "gripper_close":
            self.command.gripper = "close"

    def handle_axis(self, event: dict) -> None:
        joystick: pygame.joystick.JoystickType = self.joysticks[event["joy"]]
        mapping = self.config[joystick.get_name()]
        axes = mapping["axes"]
        axis = event["axis"]
        if axis not in axes:
            return
        movement, direction = axes[axis]["movement"], axes[axis]["direction"]
        vector = getattr(self.command, movement)
        value = event["value"]
        if "flip" in axes[axis] and axes[axis]["flip"] == "true":
            value *= -1
        if "sign" in axes[axis]:
            if axes[axis]["sign"] == "positive":
                value = (value + 1) / 2
            elif axes[axis]["sign"] == "negative":
                value = -(value + 1) / 2
        setattr(vector, direction, value)


class VirtualGamepad:
    def __init__(self) -> None:
        self.command = GamepadCommand()

    def set_callback(self, callback: Callable) -> None:
        self.callback = callback

    def run(self) -> None:
        dpg.create_context()
        dpg.create_viewport(title="Virtual Gamepad", width=400, height=400)

        with dpg.item_handler_registry(tag="handler"):
            dpg.add_item_activated_handler(callback=self.button_callback)
            dpg.add_item_deactivated_handler(callback=self.button_callback)

        self.create_window()

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()

    def create_window(self) -> None:
        window = dpg.add_window(width=-1, height=-1)
        dpg.set_primary_window(window, True)

        cols = 2
        rows = 6
        grid = dpg_grid.Grid(cols, rows, window)
        grid.offsets = 8, 8, 8, 8

        linear_buttons = ["X+", "Z+", "X-", "Z-", "Y+", "Y-"]
        other_buttons = ["G-", "G+", "Z+r", "Z-r", "Y+r", "Y-r"]
        labels = linear_buttons + other_buttons
        colors = {
            "X": (200, 0, 0),
            "Y": (0, 200, 0),
            "Z": (0, 0, 200),
            "G": (200, 200, 200),
        }
        row, col = 0, 0
        for label in labels:
            uid = dpg.generate_uuid()
            button = dpg.add_button(parent=window, label=label, tag=uid)
            dpg.bind_item_handler_registry(uid, "handler")
            self.set_color(uid, colors[label[0]])
            grid.push(button, col, row)
            col = col + 1 if col < cols - 1 else 0
            row = row + 1 if col == 0 else row

        with dpg.item_handler_registry() as window_hr:
            dpg.add_item_visible_handler(callback=grid)
        dpg.bind_item_handler_registry(window, window_hr)

    def set_color(self, tag: int, rgb: tuple) -> None:
        with dpg.theme() as item_theme, dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Button, rgb)
        dpg.bind_item_theme(tag, item_theme)

    def button_callback(self, sender: int, app_data: int) -> None:
        label = dpg.get_item_label(app_data)
        int_press = 22
        active = sender == int_press
        axis = label[0].lower()
        movement = "angular" if label[-1] == "r" else "linear"
        value = 1.0 if label[1] == "+" else -1.0
        value = 0.0 if not active else value

        if axis in ["x", "y", "z"]:
            vector = getattr(self.command, movement)
            setattr(vector, axis, value)
        elif label == "G-":
            self.command.gripper = "open"
        elif label == "G+":
            self.command.gripper = "close"
        self.callback(self.command)


if __name__ == "__main__":
    Gamepad().run()
