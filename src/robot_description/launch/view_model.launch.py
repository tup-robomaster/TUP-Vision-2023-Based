import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz2_config_path = os.path.join(get_package_share_directory('robot_description'), 'launch/view_model.rviz')
    
    use_serial = LaunchConfiguration('using_imu')
    robot_type = 'standard3' # standard hero engineer balance drone
    # robot_type = LaunchConfiguration('robot_type')
    load_chassis = LaunchConfiguration('load_chassis')
    load_gimbal = LaunchConfiguration('load_gimbal')
    load_shooter = LaunchConfiguration('load_shooter')
    load_arm = LaunchConfiguration('load_arm')
    roller_type = LaunchConfiguration('roller_type')

    # declare_robot_type = DeclareLaunchArgument(
    #     name='robot_type',
    #     default_value='standard3',
    #     description='Robot type {hero, engineer, standard}'
    # )

    declare_load_chassis = DeclareLaunchArgument(
        name='load_chassis',
        default_value='true',
        description='Whether load chassis or not.'
    )

    declare_load_gimbal = DeclareLaunchArgument(
        name='load_gimbal',
        default_value='true',
        description='Whether load gimbal or not.'
    )

    declare_load_shooter = DeclareLaunchArgument(
        name='load_shooter',
        default_value='true',
        description='Whether load shooter or not.'
    )

    declare_load_arm = DeclareLaunchArgument(
        name='load_arm',
        default_value='false',
        description='Whether load arm or not.'
    )
    
    declare_roller_type = DeclareLaunchArgument(
        name='roller_type',
        default_value='simple',
        description='simple or realistic'
    )

    declare_use_serial = DeclareLaunchArgument(
        name='using_imu',
        default_value='True',
        description='debug without serial port.'
    )

    serial_driver = Node(
        package='serialport',
        executable='serialport_node',
        name='serialport',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'using_port': True,
            'tracking_target': True,
            'print_serial_info': False,
            'print_referee_info': False
        }],
        condition=IfCondition(PythonExpression(["'", use_serial, "' == 'True'"]))
    )

    robot_description = Command([
        'xacro ', 
        os.path.join(get_package_share_directory('robot_description'), 'urdf', robot_type, robot_type) + '.urdf.xacro',
        ' load_chassis:=', load_chassis,
        ' load_gimbal:=', load_gimbal,
        ' load_shooter:=', load_shooter,
        ' load_arm:=', load_arm,
        ' use_simulation:=', 'false',
        ' roller_type:=', roller_type,
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz2_config_path]
    )

    return LaunchDescription([
        # declare_robot_type,
        declare_load_chassis,
        declare_load_gimbal,
        declare_load_shooter,
        declare_load_arm,
        declare_roller_type,
        declare_use_serial,

        serial_driver,
        robot_state_publisher,
        joint_state_publisher,
        rviz2,
    ])