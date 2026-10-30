from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Launch websockets and webrtc nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_drivers'),
                    'launch',
                    'drivers_all.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch URDF and TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    'launch',
                    'robot_state_publisher.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch the velocity EKF filter
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('filter_ekf'),
                    'launch',
                    'ekf_velocity.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch the NMPC nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('navigation_system'),
                    'launch',
                    'navigation.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

         # Launch the safety nodes
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('system_management'),
                    'launch',
                    'status.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch the perception nodes in Python.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('perception_stack'),
                    'launch',
                    'perception.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch the perception nodes in cpp
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cpp_nodes'),
                    'launch',
                    'perception_pipeline.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),

        # Launch nodes to save data
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('saves'),
                    'launch',
                    'saves.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),   
        
    ])

"""      
        # Launch the visualization in RViz2.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('robot_description'),
                    'launch',
                    'rviz2.launch.py'
                ])
            ]),
            launch_arguments={}.items()
        ),    
"""