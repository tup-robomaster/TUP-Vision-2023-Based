'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-10-25 23:10:41
LastEditTime: 2022-10-25 23:24:15
FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_processor/launch/armor_processor.launch.py
'''
import os
from sys import exec_prefix

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.action import DeclareLaunchArgument
from launch.substitution import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_path = get_package_share_directory("global_user")

    cam_config = os.path.join(share_path, "config/camera_ros.yaml")
    autoaim_config = os.path.join(share_path, "config/autoaim.yaml")

    return LaunchDescription([
        Node
        (
            name = "camera_driver",
            package = "camera_driver",
            executable = "hik_cam_node",
            parameters = [cam_config],
            output = "screen"
        ),

        Node
        (
            name = "armor_detector",
            package = "armor_detector",
            executable = "armor_detector_node",
            parameters = [autoaim_config],
            output = "screen" 
        ),

        Node
        (
            name = "armor_processor",
            package = "armor_processor",
            executable = "armor_processor_node",
            parameters = [autoaim_config],
            output = "screen"  
        )
    ])