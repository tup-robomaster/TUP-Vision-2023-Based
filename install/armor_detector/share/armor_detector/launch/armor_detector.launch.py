'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-11-08 17:54:17
LastEditTime: 2022-11-09 20:23:53
FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/launch/armor_detector.launch.py
'''
from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
    ld = LaunchDescription()

    share_path = get_package_share_directory('global_user')

    cam_config = os.path.join(share_path, 'config', 'camera_ros.yaml')
    autoaim_config = os.path.join(share_path, 'config', 'autoaim.yaml')

    # cam_node = Node(
    #     name = 'hik_cam_driver',
    #     package = 'camera_driver',
    #     executable = 'hik_cam_driver_node',
    #     parameters = [cam_config],
    #     output = 'screen'
    # )

    cam_node = Node(
        name = "daheng_cam_driver",
        package = "camera_driver",
        executable = "daheng_cam_driver_node",
        parameters = [cam_config],
        output = "screen"
    )

    armor_detector_node = Node(
        name = 'armor_detector',
        package = "armor_detector",
        executable = 'armor_detector_node',
        parameters = [autoaim_config],
        output = 'screen'
    )

    ld.add_action(cam_node)
    ld.add_action(armor_detector_node)

    return ld





