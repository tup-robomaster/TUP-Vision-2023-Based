'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-22 01:49:00
LastEditTime: 2023-04-04 23:28:15
FilePath: /TUP-Vision-2023-Based/src/global_user/launch/autoaim_bringup.launch.py
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
    # autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    # camera_node_launch_file = os.path.join(get_package_share_directory('camera_driver'), 'launch/hik_cam_node.launch.py')
    detector_node_launch_file = os.path.join(get_package_share_directory('armor_detector'), 'launch/armor_detector.launch.py')
    processor_node_launch_file = os.path.join(get_package_share_directory('armor_processor'), 'launch/armor_processor.launch.py')
    
    camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    
    camera_type = LaunchConfiguration('camera_type')
    use_serial = LaunchConfiguration('using_imu')
    debug_pred = LaunchConfiguration("debug_pred")

    declare_camera_type = DeclareLaunchArgument(
        name='camera_type',
        default_value='daheng',
        description='hik daheng mvs usb'
    )

    declare_use_serial = DeclareLaunchArgument(
        name='using_imu',
        default_value='True',
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

    with open(autoaim_param_file, 'r') as f:
        armor_detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']
    with open(autoaim_param_file, 'r') as f:
        armor_processor_params = yaml.safe_load(f)['/armor_processor']['ros__parameters']
    
    return LaunchDescription([
        declare_camera_type,
        declare_use_serial,
        declare_debug_pred,

        Node(
            package='serialport',
            executable='serialport_node',
            name='serialport',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'using_port': True,
                'tracking_target': False,
                'print_serial_info': False,
                'print_referee_info': False
            }],
            condition=IfCondition(PythonExpression(["'", use_serial, "' == 'True'"]))
        ),
        
        ComposableNodeContainer(
            name='serial_processor_container',
            package='rclcpp_components',
            executable='component_container',
            namespace='',
            output='screen',
            condition=IfCondition(PythonExpression(["'", debug_pred, "' == 'True'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='serialport',
                    plugin='serialport::SerialPortNode',
                    name='serialport',
                    parameters=[{
                        'using_port': True,
                        'tracking_target': False,
                        'print_serial_info': False,
                        'print_referee_info': False            
                    }],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='armor_processor',
                    plugin='armor_processor::ArmorProcessorNode',
                    name='armor_processor',
                    parameters=[armor_processor_params], 
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
            ]
        ),
        
        ComposableNodeContainer(
            name='armor_detector_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
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
                    package='armor_detector',
                    plugin='armor_detector::DetectorNode',
                    name='armor_detector',
                    parameters=[armor_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='armor_processor',
                #     plugin='armor_processor::ArmorProcessorNode',
                #     name='armor_processor',
                #     namespace='',
                #     parameters=[armor_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # ),  
            ],
        ),
        
        ComposableNodeContainer(
            name='armor_detector_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
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
                    package='armor_detector',
                    plugin='armor_detector::DetectorNode',
                    name='armor_detector',
                    parameters=[armor_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='armor_processor',
                #     plugin='armor_processor::ArmorProcessorNode',
                #     name='armor_processor',
                #     namespace='',
                #     parameters=[armor_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # ),  
            ],
        ),

        ComposableNodeContainer(
            name='armor_detector_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
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
                    package='armor_detector',
                    plugin='armor_detector::DetectorNode',
                    name='armor_detector',
                    parameters=[armor_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }],
                ),
                # ComposableNode(
                #     package='armor_processor',
                #     plugin='armor_processor::ArmorProcessorNode',
                #     name='armor_processor',
                #     namespace='',
                #     parameters=[armor_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # ),  
            ],
        ),

        ComposableNodeContainer(
            name='armor_detector_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
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
                    package='armor_detector',
                    plugin='armor_detector::DetectorNode',
                    name='armor_detector',
                    parameters=[armor_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                # ComposableNode(
                #     package='armor_processor',
                #     plugin='armor_processor::ArmorProcessorNode',
                #     name='armor_processor',
                #     namespace='',
                #     parameters=[armor_processor_params],
                #     extra_arguments=[{
                #         'use_intra_process_comms':True
                #     }]
                # ),  
            ],
        ),

        Node(
            package='armor_processor',
            executable='armor_processor_node',
            namespace='armor_processor',
            output='screen',
            emulate_tty=True,
            parameters=[armor_processor_params],
            condition=IfCondition(PythonExpression(["'", debug_pred, "' == 'False'"]))
        ),
    ])