'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-19 23:26:59
LastEditTime: 2022-12-20 18:36:28
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
    camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    buff_param_file = os.path.join(get_package_share_directory('global_user'), 'config/buff.yaml')
    
    armor_camera_remappings = [("/image", "/daheng_img")]
    buff_camera_remappings = [("/image", "/daheng_img")]

    camera_name = 'KE0200110074'
    use_imu = False
    bullet_speed = 25.5
    shoot_delay = 80.0 # 发弹延迟
    delay_coeff = 1.0 

    return LaunchDescription([
        # Node(
        #     package="camera_driver",
        #     executable="daheng_cam_node",
        #     name="daheng_cam_driver",
        #     output="screen",
        #     emulate_tty=True
        # ),

        Node(
            package="buff_detector",
            executable="buff_detector_node",
            name="buff_detector",
            output="screen",
            emulate_tty=True,
            remappings=buff_camera_remappings,
            parameters=[buff_param_file,
            {
                'camera_name': camera_name,
                'use_imu': use_imu
            }],
        )
    ])