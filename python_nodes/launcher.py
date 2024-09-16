#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import psutil
from os import listdir
from importlib.util import spec_from_file_location, module_from_spec
import dearpygui.dearpygui as dpg
from subprocess import Popen, check_output
from threading import Thread
from time import sleep
from rcdt_utilities.launch_utils import LaunchArguments, get_package_path, get_file_path


class Launcher:
    def __init__(self) -> None:
        self.widgets = {}
        self.ARGS = LaunchArguments()
        self.create_window()
        self.create_launch_file_selector()
        self.create_buttons()
        self.create_launch_options_selector()
        self.create_active_nodes_panel()
        self.run_gui()

    def create_window(self) -> None:
        dpg.create_context()
        dpg.create_viewport(title="Launcher", width=200, height=200)
        self.window = dpg.add_window(width=-1, height=-1)
        dpg.set_primary_window(self.window, True)

    def create_launch_file_selector(self) -> None:
        self.package = "rcdt_franka"
        launch_path = f"{get_package_path(self.package)}/launch"
        launch_files = [f for f in listdir(launch_path) if f.endswith(".launch.py")]
        launch_file = launch_files[0]

        group = dpg.add_group(parent=self.window, horizontal=True)
        dpg.add_text(parent=group, default_value=f"{self.package}")
        dpg.add_combo(
            parent=group,
            items=launch_files,
            default_value=launch_file,
            callback=lambda item: self.select_launch_file(dpg.get_value(item)),
        )
        self.launch_file = launch_file

    def create_buttons(self) -> None:
        dpg.add_spacer(parent=self.window, height=10)
        group = dpg.add_group(parent=self.window, horizontal=True)
        dpg.add_button(parent=group, label="Start", callback=self.start)
        dpg.add_button(parent=group, label="Stop", callback=self.stop)
        dpg.add_button(parent=group, label="Clear", callback=self.clear)

    def create_launch_options_selector(self) -> None:
        dpg.add_spacer(parent=self.window, height=10)
        dpg.add_text(parent=self.window, default_value="Launch options:")
        uid = dpg.generate_uuid()
        dpg.add_group(parent=self.window, tag=uid)
        self.widgets["launch_options"] = uid
        self.get_args_from_launch_file()
        self.fill_launch_options_selector()

    def create_active_nodes_panel(self) -> None:
        dpg.add_spacer(parent=self.window, height=10)
        dpg.add_text(parent=self.window, default_value="Active nodes:")
        dpg.add_text(parent=self.window, default_value="", tag="active_nodes")
        Thread(target=self.show_active_nodes, daemon=True).start()

    def run_gui(self) -> None:
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.start_dearpygui()
        dpg.destroy_context()

    def fill_launch_options_selector(self) -> None:
        uid = self.widgets["launch_options"]
        dpg.delete_item(uid, children_only=True)
        for name, value in self.ARGS.all_values().items():
            options = self.ARGS.get_options(name)
            group = dpg.add_group(parent=uid, horizontal=True)
            dpg.add_text(parent=group, default_value=name)
            (
                dpg.add_radio_button(
                    parent=group,
                    label=name,
                    items=options,
                    default_value=value,
                    horizontal=True,
                    callback=lambda item: self.ARGS.set_value(
                        dpg.get_item_label(item), dpg.get_value(item)
                    ),
                ),
            )

    def select_launch_file(self, launch_file: str) -> None:
        self.launch_file = launch_file
        self.get_args_from_launch_file()
        self.fill_launch_options_selector()

    def get_args_from_launch_file(self) -> None:
        self.package = "rcdt_franka"
        file_path = get_file_path(self.package, ["launch"], self.launch_file)
        spec = spec_from_file_location("launch_file", file_path)
        module = module_from_spec(spec)
        spec.loader.exec_module(module)
        if hasattr(module, "ARGS"):
            self.ARGS = module.ARGS
        else:
            self.ARGS = LaunchArguments()

    def launch_command(self) -> str:
        command = f"ros2 launch {self.package} {self.launch_file}"
        for name, value in self.ARGS.all_values().items():
            command += f" {name}:='{value }'"
        return command

    def start(self) -> None:
        self.process = Popen(self.launch_command(), shell=True)

    def stop(self) -> None:
        if hasattr(self, "process"):
            self.kill(self.process.pid)

    def kill(self, proc_pid: int) -> None:
        process = psutil.Process(proc_pid)
        for proc in process.children(recursive=True):
            proc.kill()
        process.kill()

    def clear(self) -> None:
        Popen("clear", shell=True)

    def show_active_nodes(self) -> None:
        while True:
            nodes = check_output(["ros2", "node", "list"]).decode("utf-8")
            dpg.set_value("active_nodes", nodes)
            sleep(1)


if __name__ == "__main__":
    Launcher()
