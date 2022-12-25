'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-09-27 18:06:56
LastEditTime: 2022-12-23 19:22:26
FilePath: /TUP-Vision-2023-Based/src/serialport/launch/serial_driver.launch.py
'''
import os
import yaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    param_file = os.path.join(get_package_share_directory('global_user', 'config/camera_ros.yaml'))

    ld = LaunchDescription()
    ld.add_action(
        Node(
            name="serialport",
            package="serialport",
            executable="serialport_node",
            namespace="",
            output="screen",
            emulate_tty=True,
            parameters=[param_file,
                {
                    "debug_without_com": 'false',
                    "baud": 115200 
                }
            ]
        )
    )
    return ld
