'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-28 21:36:07
LastEditTime: 2022-10-23 23:58:37
FilePath: /TUP-Vision-2023/src/camera_driver/launch/usb_cam_node.launch.py
'''
from argparse import Namespace
from http.server import executable
from importlib.resources import Package
from unicodedata import name
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    share_path = get_package_share_directory('global_user')
    cam_config = os.path.join(share_path, 'config/camera_ros.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=cam_config),

        Node(
            name = "usb_cam_driver",
            package="camera_driver",
            executable="usb_cam_driver_node",
            parameters = [LaunchConfiguration('params_file')],
            namespace = "",
            output = "screen"
        )
    ])
