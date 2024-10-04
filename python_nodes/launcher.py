#!/usr/bin/env python3

# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0


import psutil
from typing import Dict
from os import listdir
import dearpygui.dearpygui as dpg
from subprocess import Popen, check_output
from threading import Thread
from time import sleep
from rcdt_utilities.launch_utils import get_package_path


class Arg:
    def __init__(self) -> None:
        self.name = None
        self.value = None
        self.choices = None

    def set_value(self, value: str) -> None:
        self.value = value


class Launcher:
    def __init__(self) -> None:
        self.widgets = {}
        self.args: Dict[str, Arg] = {}
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
        for arg in self.args.values():
            group = dpg.add_group(parent=uid, horizontal=True)
            dpg.add_text(parent=group, default_value=arg.name)
            (
                dpg.add_radio_button(
                    parent=group,
                    label=arg.name,
                    items=arg.choices,
                    default_value=arg.value,
                    horizontal=True,
                    callback=lambda item: self.args[dpg.get_item_label(item)].set_value(
                        dpg.get_value(item)
                    ),
                ),
            )

    def select_launch_file(self, launch_file: str) -> None:
        self.launch_file = launch_file
        self.get_args_from_launch_file()
        self.fill_launch_options_selector()

    def get_args_from_launch_file(self) -> None:
        self.package = "rcdt_franka"
        cmd_result = check_output(
            ["ros2", "launch", self.package, self.launch_file, "-s"]
        ).decode("utf-8")
        spaces_removed = cmd_result.replace(" ", "")
        colons_removed = spaces_removed.replace(":", "")
        string_list = colons_removed.split("\n")[1:]
        string_list = [string for string in string_list if string != ""]
        n = 0
        n_max = 2
        arg = Arg()
        for string in string_list:
            match n:
                case 0:
                    name = eval(string)
                    arg.name = name
                case 1:
                    choices = eval(string.replace("Oneof", ""))
                    arg.choices = choices
                case 2:
                    value = eval(string.replace("default", ""))
                    arg.value = value
            if n == n_max:
                self.args[arg.name] = arg
                arg = Arg()
                n = 0
            else:
                n += 1

    def launch_command(self) -> str:
        command = f"ros2 launch {self.package} {self.launch_file}"
        for arg in self.args.values():
            command += f" {arg.name}:='{arg.value}'"
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
