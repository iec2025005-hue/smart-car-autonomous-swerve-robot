import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('smart_car_description')
    pkg_gazebo = get_package_share_directory('smart_car_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'smart_car_arena.sdf')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_file = os.path.join(pkg_description, 'urdf', 'smart_car.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    headless = LaunchConfiguration('headless', default='false')

    # 1. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_path]}.items(),
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # 3. Spawn Smart Car Entity in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_smart_car',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'smart_car',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.05'
        ]
    )

    # 4. ROS-Gazebo Parameter Bridge
    bridge_topics = [
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        '/fl_steer_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/fr_steer_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/rl_steer_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/rr_steer_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/fl_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/fr_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/rl_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
        '/rr_wheel_cmd@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=bridge_topics,
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo in headless mode'),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge
    ])
