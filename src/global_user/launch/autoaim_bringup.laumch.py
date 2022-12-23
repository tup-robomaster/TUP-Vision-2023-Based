'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-22 01:49:00
LastEditTime: 2022-12-23 19:26:40
FilePath: /TUP-Vision-2023-Based/src/global_user/launch/autoaim_bringup.laumch.py
'''
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    # autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    # camera_node_launch_file = os.path.join(get_package_share_directory('camera_driver'), 'launch/hik_cam_node.launch.py')
    detector_node_launch_file = os.path.join(get_package_share_directory('armor_detector'), 'launch/armor_detector.launch.py')
    processor_node_launch_file = os.path.join(get_package_share_directory('armor_processor'), 'launch/armor_processor.launch.py')
    
    return LaunchDescription([
        # DeclareLaunchArgument(
        #     name='param_file',
        #     default_value=autoaim_param_file
        # ),

        # DeclareLaunchArgument(
        #     name='debug',
        #     default_value='true',
        # ),

        # DeclareLaunchArgument(
        #     name='detect_color',
        #     default_value='1',
        #     description='0 is blue and 1 is red.'
        # ),

        # DeclareLaunchArgument(
        #     name='forbid_prediction',
        #     default_value='false'
        # ),

        # DeclareLaunchArgument(
        #     name='camera_type',
        #     default_value='daheng',
        #     description='daheng hik mvs usb'
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         launch_file_path=camera_node_launch_file
        #     )
        # ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=detector_node_launch_file
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=processor_node_launch_file
            )
        ),

    ])