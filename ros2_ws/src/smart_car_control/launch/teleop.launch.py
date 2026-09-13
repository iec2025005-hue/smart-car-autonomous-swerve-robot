import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch teleop_twist_keyboard configured for Smart Car swerve drive.
    Topic remapping: cmd_vel -> /cmd_vel (or custom topic)
    Default speeds: linear speed 0.5 m/s, angular turn 1.0 rad/s
    """
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
    speed = LaunchConfiguration('speed', default='0.5')
    turn = LaunchConfiguration('turn', default='1.0')
    repeat_rate = LaunchConfiguration('repeat_rate', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Target velocity topic for teleop'
        ),
        DeclareLaunchArgument(
            'speed',
            default_value='0.5',
            description='Linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'turn',
            default_value='1.0',
            description='Angular turn velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'repeat_rate',
            default_value='0.0',
            description='Key repeat rate in Hz (0.0 to send only on keypress)'
        ),
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            output='screen',
            parameters=[{
                'speed': speed,
                'turn': turn,
                'repeat_rate': repeat_rate
            }],
            remappings=[
                ('cmd_vel', cmd_vel_topic)
            ]
        )
    ])
