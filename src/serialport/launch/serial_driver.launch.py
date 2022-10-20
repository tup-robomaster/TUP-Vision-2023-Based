'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-27 18:06:56
LastEditTime: 2022-09-27 19:23:37
FilePath: /tup_2023/src/serialport/launch/serial_driver.launch.py
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    ld.add_action(
        Node(
            name="serialport",
            package="serialport",
            executable="serial_driver",
            namespace="",
            output="screen"
        )
    )
    return ld
