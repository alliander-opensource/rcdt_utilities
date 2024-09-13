# SPDX-FileCopyrightText: Alliander N. V.
#
# SPDX-License-Identifier: Apache-2.0

from typing import List
import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory


def get_file_path(package: str, folders: List[str], file: str) -> str:
    package_path = get_package_share_directory(package)
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
