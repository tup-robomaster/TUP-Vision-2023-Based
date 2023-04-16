'''
# Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-27 01:40:28
LastEditTime: 2023-04-16 13:32:13
FilePath: /TUP-Vision-2023-Based/src/global_user/launch/buff_bringup.launch.py
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
    detect_node_launch_file = os.path.join(get_package_share_directory('buff_detector'), 'launch/buff_detector.launch.py')
    processor_node_launch_file = os.path.join(get_package_share_directory('buff_processor'), 'launch/buff_processor.launch.py')

    camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    buff_param_file = os.path.join(get_package_share_directory('global_user'), 'config/buff.yaml')
    
    camera_type = LaunchConfiguration('camera_type')
    use_serial = LaunchConfiguration('using_imu')
    debug_pred = LaunchConfiguration("debug_pred")

    declare_camera_type = DeclareLaunchArgument(
        name='camera_type',
        default_value='usb',
        description='hik daheng mvs usb'
    )

    declare_use_serial = DeclareLaunchArgument(
        name='using_imu',
        default_value='False',
        description='debug without serial port.'
    )
    
    declare_debug_pred = DeclareLaunchArgument(
        name='debug_pred',
        default_value='False',
        description='debug armor prediction.'
    )
    
    with open(camera_param_file, 'r') as f:
        usb_cam_params = yaml.safe_load(f)['/usb_cam_driver']['ros__parameters']
    with open(camera_param_file, 'r') as f:
        hik_cam_params = yaml.safe_load(f)['/hik_cam_driver']['ros__parameters']
    with open(camera_param_file, 'r') as f:
        daheng_cam_params = yaml.safe_load(f)['/daheng_cam_driver']['ros__parameters']
    with open(camera_param_file, 'r') as f:
        mvs_cam_params = yaml.safe_load(f)['/mvs_cam_driver']['ros__parameters']

    with open(buff_param_file, 'r') as f:
        buff_detector_params = yaml.safe_load(f)['/buff_detector']['ros__parameters']
    with open(buff_param_file, 'r') as f:
        buff_processor_params = yaml.safe_load(f)['/buff_processor']['ros__parameters']
    
    return LaunchDescription([
        declare_camera_type,
        declare_use_serial,
        declare_debug_pred,

        Node(
            package='serialport',
            executable='serialport_node',
            name='serialport',
            output='log',
            emulate_tty=True,
            respawn=True,
            parameters=[{
                'using_port': True,
                'tracking_target': False,
                'print_serial_info': False,
                'print_referee_info': False
            }],
            respawn=True,
            respawn_delay=4,
            condition=IfCondition(PythonExpression(["'", use_serial, "' == 'True'"]))
        ),
        
        ComposableNodeContainer(
            name='serial_processor_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='',
            output='log',
            respawn=True,
            condition=IfCondition(PythonExpression(["'", debug_pred, "' == 'True'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='serialport',
                    plugin='serialport::SerialPortNode',
                    name='serialport',
                    parameters=[{
                        'using_port': True,
                        'tracking_target': True,
                        'print_serial_info': False,
                        'print_referee_info': False            
                    }],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='buff_processor',
                    plugin='buff_processor::BuffDetectorNode',
                    name='buff_processor',
                    parameters=[buff_processor_params], 
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
            ],
            respawn=True,
            respawn_delay=4,
        ),

        ComposableNodeContainer(
            name='buff_detector_container',
            namespace='',
            output='log',
            package='rclcpp_components',
            executable='component_container',
            respawn=True,
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'usb'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::UsbCamNode',
                    name='usb_driver',
                    parameters=[usb_cam_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::BuffDetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='buff_processor',
                #     plugin='buff_processor::BuffProcessorNode',
                #     name='buff_processor',
                #     namespace='',
                #     parameters=[buff_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # )
            ],
            respawn=True,
            respawn_delay=4,
        ),

        ComposableNodeContainer(
            name='buff_detector_container',
            namespace='',
            output='log',
            package='rclcpp_components',
            executable='component_container',
            respawn=True,
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'daheng'"])),
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
                    package='buff_detector',
                    plugin='buff_detector::BuffDetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='buff_processor',
                #     plugin='buff_processor::BuffProcessorNode',
                #     name='buff_processor',
                #     namespace='',
                #     parameters=[buff_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # )
            ],
            respawn=True,
            respawn_delay=4,
        ),

        ComposableNodeContainer(
            name='buff_detector_container',
            namespace='',
            output='log',
            package='rclcpp_components',
            executable='component_container',
            respawn=True,
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'hik'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::HikCamNode',
                    name='hik_driver',
                    parameters=[hik_cam_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::BuffDetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='buff_processor',
                #     plugin='buff_processor::BuffProcessorNode',
                #     name='buff_processor',
                #     namespace='',
                #     parameters=[buff_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # )
            ],
            respawn=True,
            respawn_delay=4,
        ),

        ComposableNodeContainer(
            name='buff_detector_container',
            namespace='',
            output='log',
            package='rclcpp_components',
            executable='component_container',
            respawn=True,
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'mvs'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::MvsCamNode',
                    name='mvs_driver',
                    parameters=[mvs_cam_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::BuffDetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='buff_processor',
                #     plugin='buff_processor::BuffProcessorNode',
                #     name='buff_processor',
                #     namespace='',
                #     parameters=[buff_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # )
            ],
            respawn=True,
            respawn_delay=4,
        ),

        Node(
            package='buff_processor',
            executable='buff_processor_node',
            output='log',
            emulate_tty=True,
            respawn=True,
            parameters=[buff_processor_params],
            respawn=True,
            respawn_delay=4,
            condition=IfCondition(PythonExpression(["'", debug_pred, "' == 'False'"]))
        )
    ])