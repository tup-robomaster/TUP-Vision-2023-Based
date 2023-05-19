'''
Description: This is a ros-based project!
Author: Liu Biao
Date: 2022-12-22 01:49:00
LastEditTime: 2023-05-09 22:09:24
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
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

def generate_launch_description():
    # autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    detector_node_launch_file = os.path.join(get_package_share_directory('armor_detector'), 'launch/armor_detector.launch.py')
    processor_node_launch_file = os.path.join(get_package_share_directory('armor_processor'), 'launch/armor_processor.launch.py')
    
    camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    # rviz2_config_path = os.path.join(get_package_share_directory('robot_description'), 'launch/view_model.rviz')
    # urdf_model_path = os.path.join(get_package_share_directory('robot_description'), 'urdf', 'my_robot/gimbal') + '.urdf.xacro'
    
    #-------------------------------------------------------------------------------------------
    #--------------------------------------Configs----------------------------------------------
    camera_type = 'daheng'
    camera_name = 'KE0200110075'
    use_serial = False
    #------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------
    
    serial_node = []
    detector_container = []
    processor_container = []
    tf_static_node = []

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
    #---------------------------------Serial Node--------------------------------------------
    if use_serial:
        serial_node = Node(package='serialport',
                            executable='serialport_node',
                            name='serialport',
                            output='screen', # log/screen/both
                            emulate_tty=True,
                            parameters=[{
                                'using_port': True,
                                'tracking_target': True,
                                'print_serial_info': False,
                                'print_referee_info': False
                            }],
                            respawn=True,
                            respawn_delay=1)
    #---------------------------------Detector Node--------------------------------------------
    camera_params = []
    camera_plugin = ""
    camera_node = ""
    camera_remappings = []
    if camera_type == "daheng":
        camera_params = daheng_cam_params
        camera_plugin = "camera_driver::DahengCamNode"
        camera_node = "daheng_driver"
        camera_remappings = [("/image", "/daheng_img")]

    elif camera_type == "usb":
        camera_params = usb_cam_params
        camera_plugin = "camera_driver::UsbCamNode"
        camera_node = "usb_driver"
        camera_remappings = [("/image", "/usb_img")]

    elif camera_type == "mvs":
        camera_params = mvs_cam_params
        camera_plugin = "camera_driver::MvsCamNode"
        camera_node = "mvs_driver"
        camera_remappings = [("/image", "/mvs_img")]

    elif camera_type == "hik":
        camera_params = hik_cam_params
        camera_plugin = "camera_driver::HikCamNode"
        camera_node = "hik_driver"
        camera_remappings = [("/image", "/hik_img")]

    else:
        raise BaseException("Invalid Cam Type!!!") 
    detector_container = ComposableNodeContainer(
        name='armor_detector_container',
        namespace='',
        output='screen',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                plugin=camera_plugin,
                name=camera_node,
                parameters=[camera_params],
                extra_arguments=[{
                    'use_intra_process_comms': True
                }]
            ),
            ComposableNode(
                package='armor_detector',
                plugin='armor_detector::DetectorNode',
                name='armor_detector',
                parameters=[armor_detector_params,
                {
                    'camera_name': camera_name,
                }],
                remappings= camera_remappings,
                extra_arguments=[{
                    'use_intra_process_comms': True
                }]
            ),
        ],
        respawn=True,
        respawn_delay=1,
    )
    #---------------------------------Processor Node--------------------------------------------
    processor_container = ComposableNodeContainer(
        name='serial_processor_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='armor_processor',
                plugin='armor_processor::ArmorProcessorNode',
                name='armor_processor',
                parameters=[armor_processor_params,
                {
                    'camera_name': camera_name,
                }],
                remappings = camera_remappings
            ),
        ],
        respawn=True,
        respawn_delay=1,
    )
    tf_static_node = Node(package='tf2_ros',
                            executable='static_transform_publisher',
                            output='screen',
                            arguments=['-0.07705601', '-0.00966292', '0.01103587', '-0.2453373',
                                        '-1.5249719', '1.408214', 'imu_link', 'camera_link'])

    ld = LaunchDescription()
    if use_serial:
        ld.add_action(serial_node)
    
    ld.add_action(detector_container)
    ld.add_action(processor_container)
    ld.add_action(tf_static_node)

    return ld