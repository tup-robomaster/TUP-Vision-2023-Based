#  Copyright 2022 RoboMaster-OSS
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

'''
Author: holakk
Date: 2021-11-06 19:47:28
LastEditors: holakk
LastEditTime: 2021-11-07 22:22:01
Description: file content
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    param_path = os.path.join(get_package_share_directory("rmoss_daheng_driver"), "config/cam_param.yaml")

    daheng_cam_node = Node(
        package="rmoss_daheng_driver",
        executable="daheng_cam",
        name="daheng_camera",
        parameters=[param_path]
    )
    ld = LaunchDescription()
    ld.add_action(daheng_cam_node)

    return ld