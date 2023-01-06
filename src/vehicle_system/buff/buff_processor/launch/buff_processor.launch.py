'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-19 23:27:32
LastEditTime: 2023-01-06 22:23:36
FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_processor/launch/buff_processor.launch.py
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
    share_path = get_package_share_directory("global_user")
    processor_param_file = os.path.join(share_path, "config/buff.yaml")

    return LaunchDescription([
        Node(
            package="buff_processor",
            executable="buff_processor_node",
            name="buff_processor",
            output="screen",
            emulate_tty=True,
            parameters=[(processor_param_file)]
        )
    ])