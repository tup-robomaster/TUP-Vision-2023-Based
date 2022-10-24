'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-29 20:13:06
LastEditTime: 2022-10-23 20:43:00
FilePath: /TUP-Vision-2023/src/camera_driver/launch/hik_cam_node.launch.py
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_path = get_package_share_directory('global_user')
    cam_config = os.path.join(share_path, 'config', 'camera_ros.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=cam_config),
        Node(
            name="hik_cam_driver",
            package = "camera_driver",
            executable = "hik_cam_driver_node",
            parameters = [LaunchConfiguration('params_file')],
            namespace = "",    
            output = 'screen'
        )
    ])
