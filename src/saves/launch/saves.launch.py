from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='saves',
            executable='logging_manager',
            output='screen',
        ),

        Node(
            package='saves',
            executable='data_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='ekf_recorder',
            output='screen',
        ),

        Node(
            package='saves',
            executable='control_signals',
            output='screen',
        ),

        Node(
            package='saves',
            executable='perception_recorder',
            output='screen',
        ),   
        
        Node(
            package='saves',
            executable='healthy_recorder',
            output='screen',
        )
    ])
