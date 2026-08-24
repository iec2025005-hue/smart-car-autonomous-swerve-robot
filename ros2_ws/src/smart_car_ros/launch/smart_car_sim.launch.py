import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # ── Package Directories ────────────────────────────────────
    pkg_description = get_package_share_directory('smart_car_description')
    pkg_gazebo = get_package_share_directory('smart_car_gazebo')
    pkg_control = get_package_share_directory('smart_car_control')
    pkg_ros = get_package_share_directory('smart_car_ros')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ── File Paths ─────────────────────────────────────────────
    world_path = os.path.join(pkg_gazebo, 'worlds', 'smart_car_arena.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'smart_car.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'smart_car.rviz')

    # ── Launch Configurations ──────────────────────────────────
    vision_mode = LaunchConfiguration('vision_mode', default='color')
    nav_mode = LaunchConfiguration('nav_mode', default='follow')
    base_speed = LaunchConfiguration('base_speed', default='0.35')
    rviz = LaunchConfiguration('rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # ── Robot Description (Xacro) ──────────────────────────────
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

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

    # 3. Spawn Smart Car Entity
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

    # 5. Swerve Controller Node (Math, Kinematics, Odometry, TF)
    swerve_controller = Node(
        package='smart_car_control',
        executable='swerve_controller_node',
        name='swerve_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Vision Node
    vision_node = Node(
        package='smart_car_ros',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'vision_mode': vision_mode,
            'use_sim_time': True
        }]
    )

    # 7. Navigation Node
    navigation_node = Node(
        package='smart_car_ros',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        parameters=[{
            'nav_mode': nav_mode,
            'base_speed': base_speed,
            'use_sim_time': True
        }]
    )

    # 8. RViz2 Visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('vision_mode', default_value='color', description='Vision mode: yolo, color, aruco, lane'),
        DeclareLaunchArgument('nav_mode', default_value='follow', description='Navigation mode: follow, avoid, patrol, idle'),
        DeclareLaunchArgument('base_speed', default_value='0.35', description='Base navigation speed (m/s)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz2'),
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless'),

        gz_sim,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        swerve_controller,
        vision_node,
        navigation_node,
        rviz_node
    ])
