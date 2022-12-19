'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-19 23:26:59
LastEditTime: 2022-12-19 23:29:36
FilePath: /TUP-Vision-2023-Based/src/vehicle_system/buff/buff_detector/launch/buff_detector.launch.py
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
    return LaunchDescription([
        
    ])