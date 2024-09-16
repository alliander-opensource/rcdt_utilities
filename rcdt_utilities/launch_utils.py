# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

import sys
from typing import List
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory


class LaunchArguments:
    def __init__(self) -> None:
        self.values = {}
        self.options = {}

    def add_value(self, name: str, value: str, options: List[str]) -> None:
        self.set_value(name, value)
        self.options[name] = options

    def set_value(self, name: str, value: str) -> None:
        self.values[name] = value

    def get_value(self, name: str) -> str:
        return self.values[name]

    def get_options(self, name: str) -> List[str]:
        return self.options[name]

    def all_values(self) -> dict:
        return self.values

    def update_from_sys(self) -> None:
        for arg in sys.argv:
            if ":=" in arg:
                name, value = arg.split(":=")
                self.set_value(name, value)


def get_package_path(package: str) -> str:
    return get_package_share_directory(package)


def get_file_path(package: str, folders: List[str], file: str) -> str:
    package_path = get_package_path(package)
    return os.path.join(package_path, *folders, file)


def get_yaml(file_path: str) -> yaml.YAMLObject:
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def get_robot_description(xacro_path: str, xacro_arguments: dict) -> str:
    robot_description_config = xacro.process_file(xacro_path, mappings=xacro_arguments)
    return {"robot_description": robot_description_config.toxml()}
