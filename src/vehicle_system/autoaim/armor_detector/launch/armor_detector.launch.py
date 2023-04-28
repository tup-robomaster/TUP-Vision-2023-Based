'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-11-08 17:54:17
LastEditTime: 2022-11-09 20:23:53
FilePath: /TUP-Vision-2023-Based/src/vehicle_system/autoaim/armor_detector/launch/armor_detector.launch.py
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
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')

    with open(camera_param_file, 'r') as f:
        daheng_cam_params = yaml.safe_load(f)['/daheng_cam_driver']['ros__parameters']
    with open(autoaim_param_file, 'r') as f:
        armor_detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']

    return LaunchDescription([
        ComposableNodeContainer(
            name='armor_detector_container',
            namespace='',
            output='log',
            package='rclcpp_components',
            executable='component_container',
            # respawn=True,
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::DahengCamNode',
                    name='daheng_driver',
                    parameters=[daheng_cam_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='armor_detector',
                    plugin='armor_detector::DetectorNode',
                    name='armor_detector',
                    parameters=[armor_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
            ],
        ),
    ])





