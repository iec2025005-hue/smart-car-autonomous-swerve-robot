from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='smart_car_control',
            executable='swerve_controller_node',
            name='swerve_controller_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
