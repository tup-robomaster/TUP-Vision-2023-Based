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
    autoaim_param_file = os.path.join(get_package_share_directory('global_user'), 'config/autoaim.yaml')
    buff_param_file = os.path.join(get_package_share_directory('global_user'), 'config/buff.yaml')
    
    camera_type = DeclareLaunchArgument(
        name='camera_type',
        default_value='daheng',
        description='hik daheng mvs usb'
    )

    use_serial = DeclareLaunchArgument(
        name='using_imu',
        default_value='false',
        description='debug without serial port.'
    )

    autoaim_launch_file = os.path.join(get_package_share_directory('global_user'), 'launch/autoaim_bringup.launch.py')
    buff_launch_file = os.path.join(get_package_share_directory('global_user'), 'launch/buff_bringup.launch.py')
    
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
    
    with open(buff_param_file, 'r') as f:
        buff_detector_params = yaml.safe_load(f)['/buff_detector']['ros_parameters']
    with open(buff_param_file, 'r') as f:
        buff_processor_params = yaml.safe_load(f)['/buff_processor']['ros_parameters']

    return LaunchDescription([
        camera_type,
        use_serial,

        # Node(
        #     package='camera_driver',
        #     executable='daheng_cam_node',
        #     name='daheng_cam_node',
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[{
        #         'camera_type':LaunchConfiguration('camera_type'),
        #     }]
        # ),

        Node(
            package='serialport',
            executable='serialport_node',
            name='serialport',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'using_imu':LaunchConfiguration('using_imu'),
                'debug_without_com': 'false'
            }],
            condition=IfCondition(PythonExpression([LaunchConfiguration('using_imu'), "== 'true'"]))
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         launch_file_path=buff_launch_file
        #     )
        # ),

        ComposableNodeContainer(
            name='usb_cam_autoaim_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'usb'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::UsbCamNode',
                    name='usb_cam_node',
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
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::DetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                )
            ],
        ),

        ComposableNodeContainer(
            name='daheng_cam_autoaim_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'daheng'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::DahengCamNode',
                    name='daheng_cam_node',
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
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::DetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                )
            ],
        ),

        ComposableNodeContainer(
            name='hik_cam_autoaim_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'hik'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::HikCamNode',
                    name='hik_cam_node',
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
                    }]
                ),
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::DetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                )
            ],
        ),

        ComposableNodeContainer(
            name='mvs_cam_autoaim_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
            condition=IfCondition(PythonExpression(["'", camera_type, "' == 'mvs'"])),
            composable_node_descriptions=[
                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::MVSCamNode',
                    name='mvs_cam_node',
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
                ComposableNode(
                    package='buff_detector',
                    plugin='buff_detector::DetectorNode',
                    name='buff_detector',
                    parameters=[buff_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                )
            ],
        ),

        Node(
            package='armor_processor',
            executable='armor_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[armor_processor_params]
        ),

        Node(
            package='buff_processor',
            executable='buff_processor_node',
            output='screen',
            emulate_tty=True,
            parameters=[buff_processor_params]
        )

    ])