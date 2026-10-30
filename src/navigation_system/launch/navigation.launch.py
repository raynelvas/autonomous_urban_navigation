from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    home_path = os.path.expanduser("~")
    global_params_path = os.path.join(
        home_path, 
        'autonomous_navigation', 'src', 'params'
    )

    # Paths to the specific YAML files
    nmpc_yaml = os.path.join(global_params_path, 'nmpc_params.yaml')

    return LaunchDescription([
        Node(
            package='navigation_system',
            executable='nmpc_controller',
            name='lane_controller_nmpc',
            parameters=[
                nmpc_yaml    # La ruta del archivo va sola en la lista
            ],
            output='screen',    
        ),
    ])

"""
        # Lanza nodo controlador robusto
        Node(
            package='navigation_system',
            executable='robust_controller',
            output='screen',    
        ),   

"""