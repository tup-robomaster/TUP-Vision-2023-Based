import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_path = get_package_share_directory('global_user')
    cam_config = os.path.join(share_path, 'config/camera_ros.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=cam_config),
        Node(
            name="daheng_cam_driver",
            package = "camera_driver",
            executable = "daheng_cam_driver_node",
            parameters = [LaunchConfiguration('params_file')],
            namespace = "",    
            output = 'screen'
        )
    ])