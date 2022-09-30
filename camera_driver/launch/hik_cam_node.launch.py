'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-29 20:13:06
LastEditTime: 2022-09-29 20:17:48
FilePath: /tup_2023/src/camera_driver/launch/hik_cam_node.launch.py
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            name="hik_driver",
            package = "camera_driver",
            executable = "hik_cam_driver_node",
            namespace = "",    
        )
    ])
