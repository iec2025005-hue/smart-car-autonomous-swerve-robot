import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('smart_car_description')

    serial_port = LaunchConfiguration('serial_port', default='/dev/serial0')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')
    vision_mode = LaunchConfiguration('vision_mode', default='yolo')
    nav_mode = LaunchConfiguration('nav_mode', default='follow')

    xacro_file = os.path.join(pkg_description, 'urdf', 'smart_car.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Vision Node (Hardware camera)
    vision_node = Node(
        package='smart_car_ros',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'vision_mode': vision_mode,
            'use_sim_time': False
        }]
    )

    # 3. Navigation Node
    navigation_node = Node(
        package='smart_car_ros',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'nav_mode': nav_mode,
            'use_sim_time': False
        }]
    )

    # 4. ESP32 Hardware Serial Bridge Node
    esp32_bridge_node = Node(
        package='smart_car_ros',
        executable='esp32_bridge_node',
        name='esp32_bridge_node',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/serial0', description='ESP32 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200', description='Serial baud rate'),
        DeclareLaunchArgument('vision_mode', default_value='yolo', description='Vision mode: yolo, color, aruco, lane'),
        DeclareLaunchArgument('nav_mode', default_value='follow', description='Navigation mode: follow, avoid, patrol, idle'),

        robot_state_publisher,
        vision_node,
        navigation_node,
        esp32_bridge_node
    ])
