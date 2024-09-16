# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import Callable

import dearpygui.dearpygui as dpg
import dearpygui_grid as dpg_grid


def default_callback(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
    """Print the input variables."""
    print(f"x: {x}, y: {y}, z: {z}")


class VirtualGamepad:
    """A gamepad simulation using a GUI created by dearpygui."""

    def __init__(self) -> None:
        """Define callback and inputs."""
        self.callback = default_callback
        self.x = 0
        self.y = 0
        self.z = 0
        self.g = 0

    def set_callback(self, callback: Callable) -> None:
        self.callback = callback

    def run(self) -> None:
        """Run the virtual gamepad."""
        dpg.create_context()
        dpg.create_viewport(title="Virtual Gamepad", width=400, height=400)

        self.create_handler()
        self.create_window()

        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()

    def create_handler(self) -> None:
        """Create a handler for item activation and deactivation."""
        with dpg.item_handler_registry(tag="handler"):
            dpg.add_item_activated_handler(callback=self.button_callback)
            dpg.add_item_deactivated_handler(callback=self.button_callback)

    def create_window(self) -> None:
        """Create the window as a grid of buttons."""
        window = dpg.add_window(width=-1, height=-1)
        dpg.set_primary_window(window, True)

        cols = 2
        rows = 4
        grid = dpg_grid.Grid(cols, rows, window)
        grid.offsets = 8, 8, 8, 8

        labels = ["X+", "Z+", "X-", "Z-", "Y+", "Y-", "G+", "G-"]
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
        """Set the color of an item given by tag."""
        with dpg.theme() as item_theme, dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_Button, rgb)
        dpg.bind_item_theme(tag, item_theme)

    def button_callback(self, sender: int, app_data: int) -> None:
        """Handle button press/release and call main callback function."""
        label = dpg.get_item_label(app_data)
        int_press = 22
        active = sender == int_press
        axis = label[0].lower()
        value = 1 if label[1] == "+" else -1
        value = 0 if not active else value
        setattr(self, axis, value)
        self.callback(self.x, self.y, self.z, self.g)


if __name__ == "__main__":
    VirtualGamepad().run()
