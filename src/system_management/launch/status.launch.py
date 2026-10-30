import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Paths to specific YAML files
    laptop_limits_yaml = os.path.join(global_params_path, 'laptop_limits.yaml')
    battery_limits_yaml = os.path.join(global_params_path, 'battery_limits.yaml')

    return LaunchDescription([        
        
        Node(
            package='system_management',
            executable='master_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='system_health',
            name='system_health_monitor',
            parameters=[laptop_limits_yaml],
            output='screen'
        ),

        Node(
            package='system_management',
            executable='camera_supervisor',
            output='screen'
        ),

        Node(
            package='system_management',
            executable='VC_supervisor',
            name='supervisor_voltage',
            parameters=[battery_limits_yaml],
            output='screen'
        ),

        Node(
            package='system_management',
            executable='esp32_safety',
            output='screen',
        ),

        Node
        (
            package='system_management',
            executable='robot_lights_controller',
            output='screen',
        ),
        
    ])