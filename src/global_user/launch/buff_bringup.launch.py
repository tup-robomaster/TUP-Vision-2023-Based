import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    detect_node_launch_file = os.path.join(get_package_share_directory('buff_detector'), 'launch/buff_detector.launch.py')
    processor_node_launch_file = os.path.join(get_package_share_directory('buff_processor'), 'launch/buff_processor.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=detect_node_launch_file
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                launch_file_path=processor_node_launch_file
            )
        )
    ])