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
    
    #-------------------------------------------------------------------------------------------
    #--------------------------------------Configs----------------------------------------------
    camera_type = 'daheng' # (daheng/hik/mvs/usb)
    camera_name = 'KE0200110076'
    use_serial = True
    use_imu = True
    bullet_speed = 14.5 # 弹速
    shoot_delay = 150.0 # 发弹延迟
    delay_coeff = 1.0   # 延迟系数（放大时间提前量，缓解云台跟随滞后问题
    #------------------------------------------------------------------------------------------
    #------------------------------------------------------------------------------------------
    
    serial_node = []
    detector_container = []
    # processor_container = []
    armor_processor_node = []
    buff_processor_node = []
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
    
    with open(buff_param_file, 'r') as f:
        buff_detector_params = yaml.safe_load(f)['/buff_detector']['ros__parameters']
    with open(buff_param_file, 'r') as f:
        buff_processor_params = yaml.safe_load(f)['/buff_processor']['ros__parameters']
        
    # tf2 node    
    tf_static_node = Node(package='tf2_ros',
                            executable='static_transform_publisher',
                            output='screen',
                            arguments=['-0.07705601', '-0.00966292', '0.01103587', '-0.2453373',
                                        '-1.5249719', '1.408214', 'imu_link', 'camera_link'])
    
    #---------------------------------Serial Node--------------------------------------------
    if use_serial:
        serial_node = Node(package='serialport',
                            executable='serialport_node',
                            name='serialport',
                            output='screen', # log/screen/both
                            emulate_tty=True,
                            parameters=[{
                                'using_port': True,
                                'tracking_target': False,
                                'print_serial_info': False,
                                'print_referee_info': False
                            }],
                            respawn=True,
                            respawn_delay=1)

    
    #---------------------------------Detector Node--------------------------------------------
    camera_params = []
    camera_plugin = ""
    camera_node = ""

    armor_camera_remappings = []
    buff_camera_remappings = []
    if camera_type == "daheng":
        camera_params = daheng_cam_params
        camera_plugin = "camera_driver::DahengCamNode"
        camera_node = "daheng_driver"
        armor_camera_remappings = [("/image", "/daheng_img_armor_node")]
        buff_camera_remappings = [("/image", "/daheng_img_buff_node")]

    elif camera_type == "usb":
        camera_params = usb_cam_params
        camera_plugin = "camera_driver::UsbCamNode"
        camera_node = "usb_driver"
        armor_camera_remappings = [("/image", "/usb_img_armor_node")]
        buff_camera_remappings = [("/image", "/usb_img_buff_node")]

    elif camera_type == "mvs":
        camera_params = mvs_cam_params
        camera_plugin = "camera_driver::MvsCamNode"
        camera_node = "mvs_driver"
        armor_camera_remappings = [("/image", "/mvs_img_armor_node")]
        buff_camera_remappings = [("/image", "/mvs_img_buff_node")]

    elif camera_type == "hik":
        camera_params = hik_cam_params
        camera_plugin = "camera_driver::HikCamNode"
        camera_node = "hik_driver"
        armor_camera_remappings = [("/image", "/hik_img_armor_node")]
        buff_camera_remappings = [("/image", "/hik_img_buff_node")]

    else:
        raise BaseException("Invalid Cam Type!!!") 
    
    detector_container = ComposableNodeContainer(
        name='detector_container',
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
                    'use_serial': use_serial,
                    'use_imu': use_imu,
                    'bullet_speed': bullet_speed,
                }],
                remappings= armor_camera_remappings,
                extra_arguments=[{
                    'use_intra_process_comms': True
                }]
            ),
            ComposableNode(
                package='buff_detector',
                plugin='buff_detector::BuffDetectorNode',
                name='buff_detector',
                parameters=[buff_detector_params,
                {
                    'camera_name': camera_name,
                    'use_serial': use_serial,
                    'use_imu': use_imu,
                    'bullet_speed': bullet_speed,
                }],
                remappings= buff_camera_remappings,
                extra_arguments=[{
                    'use_intra_process_comms': True
                }]
            ),
        ],
        respawn=True,
        respawn_delay=1,
    )
    #---------------------------------Processor Node--------------------------------------------
    armor_processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        name='armor_processor',
        output='screen',
        emulate_tty=True,
        parameters=[armor_processor_params,
        {
            'camera_name': camera_name,
            'bullet_speed': bullet_speed,
            'shoot_delay': shoot_delay,
            'delay_coeff': delay_coeff,
        }],
        remappings = armor_camera_remappings,
        respawn=True,
        respawn_delay=1
    )
    
    buff_processor_node = Node(
        package='buff_processor',
        executable='buff_processor_node',
        name='buff_processor',
        output='screen', # log/screen/both
        emulate_tty=True,
        parameters=[buff_processor_params,
        {
            'camera_name': camera_name,
            'bullet_speed': bullet_speed,
            'shoot_delay': shoot_delay,
            'delay_coeff': delay_coeff,
        }],
        remappings = buff_camera_remappings,
        respawn=True,
        respawn_delay=1
    )

    ld = LaunchDescription()
    if use_serial:
        ld.add_action(serial_node)
    
    ld.add_action(detector_container)
    # ld.add_action(processor_container)
    
    ld.add_action(armor_processor_node)
    ld.add_action(buff_processor_node)
    ld.add_action(tf_static_node)

    return ld