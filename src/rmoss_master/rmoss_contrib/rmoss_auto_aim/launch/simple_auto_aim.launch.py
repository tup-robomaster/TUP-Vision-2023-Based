# Copyright 2021 RoboMaster-OSS
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg_rmoss_auto_aim = get_package_share_directory('rmoss_auto_aim')
    params_file = os.path.join(pkg_rmoss_auto_aim, 'config', 'simple_auto_aim_params.yaml')
    
    # image_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'test.png')
    # calibration_path = 'package://rmoss_cam/resource/image_cam_calibration.yaml'
    # # camera node
    # virtual_image_cam = Node(
    #     package='rmoss_cam',
    #     executable='virtual_cam',
    #     name='virtual_image_cam',
    #     parameters=[
    #         {'image_path': image_path,
    #          'camera_name': 'front_camera',
    #          'camera_info_url': calibration_path,
    #          'fps': 30,
    #          'autostart': True}],
    #     output='screen'
    # )
    # ld.add_action(virtual_image_cam)

    image_path = os.path.join(get_package_share_directory('rmoss_cam'), 'resource', 'test.png')
    calibration_path = 'package://rmoss_cam/resource/image_cam_calibration.yaml'
    params = os.path.join(get_package_share_directory('rmoss_cam'), 'config', 'cam_params.yaml')
    # camera node
    usb_cam = Node(
        package='rmoss_cam',
        executable='usb_cam',
        name='usb_cam',
        parameters=[
            {
             'camera_name': 'front_camera',
             'camera_info_url': calibration_path,
             'fps':30,
             'autostart': True}],
        output='screen'
    )
    ld.add_action(usb_cam)

    # calibration_path = os.path.join(get_package_share_directory('rmoss_daheng_driver'), 'config','cam_param.yaml')
    # # camera node
    # daheng_cam_node = Node(
    #     package="rmoss_daheng_driver",
    #     executable="daheng_cam",
    #     name="daheng_camera",
    #     parameters=[calibration_path],
    #     output='screen'
    # )
    # ld.add_action(daheng_cam_node)

    auto_aim_node = Node(
        package='rmoss_auto_aim',
        executable='simple_auto_aim',
        parameters=[params_file],
        output='screen'
    )
    ld.add_action(auto_aim_node)
    faker_tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        arguments=['0', '0', '0', '0.0', '0.0', '0.0',
                   'base_link', 'gimbal_yaw']
    )
    faker_tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        arguments=['0', '0', '0.1', '0.0', '0.0', '0.0',
                   'gimbal_yaw', 'gimbal_pitch']
    )
    faker_tf3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        arguments=['0', '0', '0.1', '0', '0.0', '0',
                   'gimbal_yaw', 'gimbal_home']
    )
    faker_tf4 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher1',
        arguments=['0.1', '0', '0', '-1.57', '0.0', '-1.57',
                   'gimbal_pitch', 'front_camera_optical']
    )
    ld.add_action(faker_tf1)
    ld.add_action(faker_tf2)
    ld.add_action(faker_tf3)
    ld.add_action(faker_tf4)
    return ld
