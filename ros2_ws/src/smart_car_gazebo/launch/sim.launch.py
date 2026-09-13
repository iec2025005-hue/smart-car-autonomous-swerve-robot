import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_description = get_package_share_directory('smart_car_description')
    pkg_gazebo = get_package_share_directory('smart_car_gazebo')
    pkg_control = get_package_share_directory('smart_car_control')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'smart_car_arena.sdf')
    xacro_file = os.path.join(pkg_description, 'urdf', 'smart_car.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'rviz', 'smart_car.rviz')

    # Set resource search paths for Gazebo mesh loading (chassis, steer brackets, wheels)
    resource_paths = [
        os.path.dirname(pkg_description),
        os.path.dirname(pkg_gazebo),
        pkg_description,
        pkg_gazebo,
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    ]
    resource_path_str = ':'.join([p for p in resource_paths if p])

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path_str
    )
    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path_str
    )

    headless = LaunchConfiguration('headless', default='false')
    rviz = LaunchConfiguration('rviz', default='false')
    spawn_x = LaunchConfiguration('spawn_x', default='0.0')
    spawn_y = LaunchConfiguration('spawn_y', default='0.0')
    spawn_z = LaunchConfiguration('spawn_z', default='0.05')

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = ParameterValue(
        robot_description_content,
        value_type=str
    )

    # 1. Gazebo Sim Launch (GUI & Headless support)
    gz_sim_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        name='gazebo',
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': resource_path_str,
            'IGN_GAZEBO_RESOURCE_PATH': resource_path_str,
        },
        condition=IfCondition(PythonExpression(["'", headless, "' != 'true'"]))
    )

    gz_sim_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_path],
        name='gazebo',
        output='screen',
        additional_env={
            'GZ_SIM_RESOURCE_PATH': resource_path_str,
            'IGN_GAZEBO_RESOURCE_PATH': resource_path_str,
        },
        condition=IfCondition(PythonExpression(["'", headless, "' == 'true'"]))
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
            '-world', 'smart_car_world',
            '-name', 'smart_car',
            '-string', robot_description_content,
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z
        ]
    )

    # 4. ROS-Gazebo Parameter Bridge (Core telemetry & Joint Actuators)
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

    # 5. Swerve Controller Node (Kinematics, /cmd_vel subscriber, /odom publisher)
    swerve_controller = Node(
        package='smart_car_control',
        executable='swerve_controller_node',
        name='swerve_controller_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. Optional RViz2 Visualization
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
        set_gz_resource_path,
        set_ign_resource_path,
        DeclareLaunchArgument('headless', default_value='false', description='Run Gazebo headless'),
        DeclareLaunchArgument('rviz', default_value='false', description='Launch RViz2 display'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', description='Initial spawn X position'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', description='Initial spawn Y position'),
        DeclareLaunchArgument('spawn_z', default_value='0.05', description='Initial spawn Z position'),
        gz_sim_gui,
        gz_sim_headless,
        robot_state_publisher,
        spawn_entity,
        ros_gz_bridge,
        swerve_controller,
        rviz_node
    ])
