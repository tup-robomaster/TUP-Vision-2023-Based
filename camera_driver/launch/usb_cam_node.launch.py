'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-28 21:36:07
LastEditTime: 2022-09-29 20:24:11
FilePath: /tup_2023/src/camera_driver/launch/usb_cam_node.launch.py
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

def generate_launch_description():
    return LaunchDescription([
        Node(
            name = "usb_camera",
            package="camera_driver",
            executable="usb_cam_driver_node",
            namespace = ""
        )
    ])
