import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # virtual cam node
    video_path='/home/dishierweidu/Documents/TUP-Vision-main/RH.avi'
    cam_node = Node(package='rmoss_cam',
        executable='virtual_cam',
        name='virtual_video_cam',
        parameters=[{'video_path': video_path,
                    'camera_name': 'front_camera',
                    'camera_k': [1552.7, 0.0, 640.0, 0.0, 1537.9, 360.0, 0.0, 0.0, 1.0],
                    'camera_d': [0.0, 0.0, 0.0, 0.0, 0.0]}],
        output='screen'
    )
    # task auto aim node
    params_file = os.path.join(get_package_share_directory('rmoss_power_rune2019'),
        'config', 'power_rune_params.yaml')
    power_rune_node = Node(
        package='rmoss_power_rune2019',
        executable='power_rune',
        name="simple_power_rune",
        parameters=[params_file],
        output='screen'
    )
    # add node
    ld = LaunchDescription()
    ld.add_action(cam_node)
    ld.add_action(power_rune_node)
    return ld
