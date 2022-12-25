'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-19 23:27:32
LastEditTime: 2022-12-21 17:13:58
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
    processor_param_file = os.path.join(get_package_share_directory("global_user"), "config/buff.yaml")

    return LaunchDescription([
        Node(
            package="buff_processor",
            executable="buff_processor_node",
            name="buff_processor",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration(processor_param_file),
            {
                "debug": "true",
                "using_imu": "false"
            }]
        )
    ])